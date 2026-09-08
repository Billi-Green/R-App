#include "../sdk/ghostesp_plugin_api.h"
#include "../sdk/ghostesp_helpers.h"

#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RADAR_MAX_NODES 16
#define RADAR_MAX_TARGETS 3
#define RADAR_CONNECT_ROW_POOL 2
#define RADAR_CHANNEL 1
#define RADAR_MAX_RANGE_MM 8000
#define RADAR_RANGE_RINGS 4
#define RADAR_HALF_ANGLE_DEG 40
#define RADAR_CONE_SIN_MILLI 643
#define RADAR_CONE_COS_MILLI 766
#define RADAR_SWEEP_STEP_MS 16
#define RADAR_SWEEP_STEP_DEG 1
#define RADAR_DATA_TIMEOUT_MS 1200
#define RADAR_PEER_TIMEOUT_MS 10000
#define RADAR_HELLO_INTERVAL_MS 2000
#define RADAR_CONE_GREEN 0x00E060
#define RADAR_GRID_GREEN 0x008A3D
#define RADAR_TARGET_RED 0xFF3030
#define RADAR_CONNECTED_GREEN RADAR_CONE_GREEN
#define RADAR_CHECK_FALLBACK "v"

typedef enum {
    PAGE_HOME,
    PAGE_CONNECT,
    PAGE_RADAR,
} radar_page_t;

typedef struct {
    bool detected;
    int32_t x_mm;
    int32_t y_mm;
    int32_t speed;
    int32_t distance_mm;
    int32_t angle_deg;
    uint32_t last_data_ms;
} radar_target_t;

typedef struct {
    bool used;
    bool selected;
    bool streaming;
    uint8_t mac[6];
    int8_t rssi;
    char name[GHOSTESP_ESPNOW_NAME_MAX];
    uint32_t last_peer_ms;
    uint32_t last_data_ms;
    uint32_t sequence;
    radar_target_t targets[RADAR_MAX_TARGETS];
} radar_node_t;

static const ghostesp_api_t *api;
static ghostesp_theme_t theme;
static ghostesp_layout_t layout;
static ghostesp_ui_obj_t screen;
static ghostesp_ui_obj_t app_root;
static ghostesp_ui_obj_t home_screen;
static ghostesp_ui_obj_t connect_screen;
static ghostesp_ui_obj_t radar_screen;
static ghostesp_ui_obj_t canvas;
static ghostesp_ui_obj_t sweep_line;
static ghostesp_ui_obj_t target_markers[RADAR_MAX_TARGETS];
static ghostesp_ui_obj_t range_labels[RADAR_RANGE_RINGS];
static ghostesp_ui_obj_t connect_list;
static ghostesp_ui_obj_t connect_rows[RADAR_MAX_NODES];
static ghostesp_ui_obj_t connect_empty_label;
static ghostesp_ui_obj_t connect_buttons[RADAR_MAX_NODES];
static ghostesp_ui_obj_t connect_checks[RADAR_MAX_NODES];
static bool connect_row_visible[RADAR_MAX_NODES];
static bool connect_empty_visible;
static bool connect_check_is_line[RADAR_MAX_NODES];
static bool connect_check_state[RADAR_MAX_NODES];
static bool connect_check_state_valid[RADAR_MAX_NODES];
static int connect_rendered_count;
static bool connect_ui_dirty;
static ghostesp_ui_obj_t status_label;
static ghostesp_ui_obj_t home_status_label;
static ghostesp_ui_obj_t radar_label;
static ghostesp_ui_obj_t radar_data_label;
static radar_node_t nodes[RADAR_MAX_NODES];
static int node_count;
static int view_slot;
static radar_page_t page;
static uint32_t last_announce_ms;
static bool radar_background_drawn;
static int32_t radar_canvas_width;
static int32_t radar_canvas_height;
static bool marker_visible[RADAR_MAX_TARGETS];
static int marker_x[RADAR_MAX_TARGETS];
static int marker_y[RADAR_MAX_TARGETS];
static int sweep_angle_deg;
static int sweep_direction;
static uint32_t sweep_elapsed_ms;
static int sweep_drawn_angle_deg;

char *strcpy(char *dst, const char *src) __attribute__((weak));
char *strcpy(char *dst, const char *src) {
    char *out = dst;
    while ((*dst++ = *src++) != '\0') {}
    return out;
}

static void show_home(void);
static void show_connect(void);
static void show_connect_page(void);
static void show_radar(void);
static void update_node_button(int index);
static void refresh_connect_rows(void);

static void radar_logf(const char *format, ...) {
    if (!api || !api->log || !format) return;
    char message[192];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    api->log(message);
}

static uint32_t now_ms(void) {
    return api && api->system_uptime_ms ? api->system_uptime_ms() : 0;
}

static bool is_radar_name(const char *name) {
    if (!name) return false;
    return strncmp(name, "RD03D-", 6) == 0 ||
           strncmp(name, "RD-03D", 6) == 0;
}

static int find_node(const uint8_t mac[6]) {
    for (int i = 0; i < node_count; ++i) {
        if (!nodes[i].used) continue;
        bool equal = true;
        for (int byte = 0; byte < 6; ++byte) {
            if (nodes[i].mac[byte] != mac[byte]) {
                equal = false;
                break;
            }
        }
        if (equal) return i;
    }
    return -1;
}

static int ensure_node(const uint8_t mac[6], const char *name) {
    int index = find_node(mac);
    if (index >= 0) return index;
    if (node_count >= RADAR_MAX_NODES) return -1;

    index = node_count++;
    memset(&nodes[index], 0, sizeof(nodes[index]));
    nodes[index].used = true;
    memcpy(nodes[index].mac, mac, 6);
    snprintf(nodes[index].name, sizeof(nodes[index].name), "%s",
             name && name[0] ? name : "RD03D Radar");
    return index;
}

static int selected_count(void) {
    int count = 0;
    for (int i = 0; i < node_count; ++i) {
        if (nodes[i].used && nodes[i].selected) ++count;
    }
    return count;
}

static bool node_is_connected(const radar_node_t *node) {
    return node && node->used && node->streaming;
}

static int connected_count(void) {
    int count = 0;
    for (int i = 0; i < node_count; ++i) {
        if (nodes[i].used && node_is_connected(&nodes[i])) ++count;
    }
    return count;
}

static void update_connect_status(void) {
    if (!status_label || !api->ui_label_set_text) return;

    char status[96];
    if (layout.content_w < 180) {
        snprintf(status, sizeof(status), "Radars %d  |  Sel %d  |  On %d",
                 node_count, selected_count(), connected_count());
    } else {
        snprintf(status, sizeof(status),
                 "ESP-NOW ready  |  %d radar%s  |  Sel %d  |  On %d",
                 node_count, node_count == 1 ? "" : "s", selected_count(),
                 connected_count());
    }
    api->ui_label_set_text(status_label, status);
}

static int selected_node_at(int slot) {
    int current = 0;
    for (int i = 0; i < node_count; ++i) {
        if (!nodes[i].used || !nodes[i].selected) continue;
        if (current++ == slot) return i;
    }
    return -1;
}

static void delete_screen(void) {
    if (home_screen && api->ui_obj_delete) api->ui_obj_delete(home_screen);
    if (connect_screen && api->ui_obj_delete) api->ui_obj_delete(connect_screen);
    if (radar_screen && api->ui_obj_delete) api->ui_obj_delete(radar_screen);
    screen = NULL;
    home_screen = NULL;
    connect_screen = NULL;
    radar_screen = NULL;
    canvas = NULL;
    sweep_line = NULL;
    for (int i = 0; i < RADAR_MAX_TARGETS; ++i) {
        target_markers[i] = NULL;
        marker_visible[i] = false;
        marker_x[i] = 0;
        marker_y[i] = 0;
    }
    for (int i = 0; i < RADAR_RANGE_RINGS; ++i) range_labels[i] = NULL;
    connect_list = NULL;
    connect_empty_label = NULL;
    connect_empty_visible = false;
    for (int i = 0; i < RADAR_MAX_NODES; ++i) {
        connect_rows[i] = NULL;
        connect_buttons[i] = NULL;
        connect_checks[i] = NULL;
        connect_row_visible[i] = false;
        connect_check_is_line[i] = false;
        connect_check_state[i] = false;
        connect_check_state_valid[i] = false;
    }
    connect_rendered_count = 0;
    connect_ui_dirty = false;
    status_label = NULL;
    home_status_label = NULL;
    radar_label = NULL;
    radar_data_label = NULL;
    radar_background_drawn = false;
    radar_canvas_width = 0;
    radar_canvas_height = 0;
    sweep_angle_deg = 0;
    sweep_direction = 1;
    sweep_elapsed_ms = 0;
    sweep_drawn_angle_deg = -1;
}

static void style_label(ghostesp_ui_obj_t label, uint32_t color,
                        ghostesp_font_size_t font) {
    if (!label) return;
    if (api->ui_obj_set_text_color) api->ui_obj_set_text_color(label, color);
    if (api->ui_obj_set_font) api->ui_obj_set_font(label, font);
}

static ghostesp_ui_obj_t add_label(ghostesp_ui_obj_t parent, const char *text,
                                   ghostesp_font_size_t font, uint32_t color) {
    if (!api->ui_label_create) return NULL;
    ghostesp_ui_obj_t label = api->ui_label_create(parent, text);
    style_label(label, color, font);
    return label;
}

static ghostesp_ui_obj_t add_button(ghostesp_ui_obj_t parent, const char *text,
                                    ghostesp_ui_button_cb_t callback,
                                    void *user) {
    if (!api->ui_button_create) return NULL;
    ghostesp_ui_obj_t button = api->ui_button_create(parent, text, callback, user);
    if (button) {
        if (api->ui_obj_set_width) api->ui_obj_set_width(button, layout.content_w - 16);
        if (api->ui_obj_set_text_color) api->ui_obj_set_text_color(button, theme.text);
        if (api->ui_obj_set_scrollable) api->ui_obj_set_scrollable(button, false);
    }
    return button;
}

static ghostesp_ui_obj_t add_radar_button(ghostesp_ui_obj_t parent, const char *text,
                                          ghostesp_ui_button_cb_t callback,
                                          int32_t width) {
    ghostesp_ui_obj_t button = add_button(parent, text, callback, NULL);
    if (!button) return NULL;
    if (api->ui_obj_set_width) api->ui_obj_set_width(button, width);
    if (api->ui_obj_set_pad) api->ui_obj_set_pad(button, 3, 3, 2, 2);
    return button;
}

static void style_flat_container(ghostesp_ui_obj_t container,
                                 ghostesp_flex_flow_t flow) {
    if (!container) return;
    if (api->ui_obj_set_bg_color) api->ui_obj_set_bg_color(container, theme.bg);
    if (api->ui_obj_set_border_width) api->ui_obj_set_border_width(container, 0);
    if (api->ui_obj_set_radius) api->ui_obj_set_radius(container, 0);
    if (api->ui_obj_set_pad) api->ui_obj_set_pad(container, 0, 0, 0, 0);
    if (api->ui_obj_set_flex_flow) api->ui_obj_set_flex_flow(container, flow);
    if (api->ui_obj_set_scrollable) api->ui_obj_set_scrollable(container, false);
}

static void activate_screen(ghostesp_ui_obj_t next) {
    if (screen && screen != next && api->ui_obj_set_visible)
        api->ui_obj_set_visible(screen, false);
    screen = next;
    if (screen && api->ui_obj_set_visible)
        api->ui_obj_set_visible(screen, true);
}

static ghostesp_ui_obj_t create_page_screen(void) {
    if (!app_root || !api->ui_card_create) {
        radar_logf("RD03D: page create failed root=%d card_api=%d",
                   app_root ? 1 : 0, api && api->ui_card_create ? 1 : 0);
        return NULL;
    }
    ghostesp_ui_obj_t page_screen = api->ui_card_create(app_root);
    if (!page_screen) {
        radar_logf("RD03D: page create returned NULL");
        return NULL;
    }
    style_flat_container(page_screen, GHOSTESP_FLEX_FLOW_COLUMN);
    if (api->ui_obj_set_flex_grow) api->ui_obj_set_flex_grow(page_screen, 1);
    if (api->ui_obj_set_visible) api->ui_obj_set_visible(page_screen, false);
    return page_screen;
}

static void update_home_status(void) {
    if (!home_status_label || !api->ui_label_set_text) return;
    char status[96];
    snprintf(status, sizeof(status), "%d radar%s selected", selected_count(),
             selected_count() == 1 ? "" : "s");
    api->ui_label_set_text(home_status_label, status);
}

static bool start_espnow(void) {
    if (api->espnow_is_active && api->espnow_is_active()) return true;
    if (!api->espnow_start || !api->espnow_start(RADAR_CHANNEL)) {
        const char *error = api->espnow_last_error ? api->espnow_last_error() : "ESP-NOW unavailable";
        if (api->toast) api->toast(error);
        return false;
    }
    if (api->espnow_announce) api->espnow_announce();
    last_announce_ms = now_ms();
    return true;
}

static bool refresh_nodes(void) {
    if (!api->espnow_peer_count || !api->espnow_get_peer) return false;
    bool changed = false;
    uint32_t now = now_ms();

    int peers = api->espnow_peer_count();
    for (int i = 0; i < peers; ++i) {
        ghostesp_espnow_peer_t peer;
        if (!api->espnow_get_peer(i, &peer) || !is_radar_name(peer.name)) continue;
        int previous_count = node_count;
        int index = ensure_node(peer.mac, peer.name);
        if (index < 0) continue;
        if (node_count != previous_count) changed = true;
        if (strcmp(nodes[index].name, peer.name) != 0) changed = true;
        snprintf(nodes[index].name, sizeof(nodes[index].name), "%s", peer.name);
        nodes[index].rssi = peer.rssi;
        nodes[index].last_peer_ms = peer.last_seen_ms ? peer.last_seen_ms : now;
    }

    for (int i = node_count - 1; i >= 0; --i) {
        if (!nodes[i].used || !nodes[i].last_peer_ms ||
            now - nodes[i].last_peer_ms <= RADAR_PEER_TIMEOUT_MS) continue;
        if (!nodes[i].selected || page == PAGE_CONNECT) {
            radar_logf("RD03D: remove stale node=%d page=%d selected=%d streaming=%d age=%lu",
                       i, page, nodes[i].selected ? 1 : 0,
                       nodes[i].streaming ? 1 : 0,
                       (unsigned long)(now - nodes[i].last_peer_ms));
            memmove(&nodes[i], &nodes[i + 1],
                    (size_t)(node_count - i - 1) * sizeof(nodes[0]));
            --node_count;
            changed = true;
        }
    }
    return changed;
}

static bool parse_long_field(char **cursor, long *value) {
    if (!cursor || !*cursor || !value) return false;
    char *current = *cursor;
    bool negative = false;
    if (*current == '-') {
        negative = true;
        ++current;
    }
    if (*current < '0' || *current > '9') return false;

    long parsed = 0;
    while (*current >= '0' && *current <= '9') {
        parsed = parsed * 10 + (*current - '0');
        ++current;
    }
    *value = negative ? -parsed : parsed;
    *cursor = *current == ',' ? current + 1 : current;
    return true;
}

static bool parse_telemetry(const char *text, uint32_t *sequence,
                            uint8_t *target_id, bool *detected, int32_t *x,
                            int32_t *y, int32_t *speed, int32_t *distance,
                            int32_t *angle) {
    if (!text || strncmp(text, "RADAR,", 6) != 0) return false;

    char copy[GHOSTESP_ESPNOW_MESSAGE_MAX];
    snprintf(copy, sizeof(copy), "%s", text + 6);
    char *cursor = copy;
    long values[8];
    for (int i = 0; i < 8; ++i) {
        if (!parse_long_field(&cursor, &values[i])) return false;
    }

    *sequence = (uint32_t)values[0];
    if (values[1] < 0 || values[1] >= RADAR_MAX_TARGETS) return false;
    *target_id = (uint8_t)values[1];
    *detected = values[2] != 0;
    *x = (int32_t)values[3];
    *y = (int32_t)values[4];
    *speed = (int32_t)values[5];
    *distance = (int32_t)values[6];
    *angle = (int32_t)values[7];
    return true;
}

static void drain_messages(void) {
    if (!api->espnow_receive) return;
    ghostesp_espnow_message_t message;
    while (api->espnow_receive(&message)) {
        int index = find_node(message.sender_mac);
        if (index < 0 && is_radar_name(message.sender_name)) {
            int previous_count = node_count;
            index = ensure_node(message.sender_mac, message.sender_name);
            if (node_count != previous_count) connect_ui_dirty = true;
        }
        if (index < 0) continue;

        if (strcmp(message.text, "RADAR_ACK,START") == 0) {
            nodes[index].streaming = true;
            connect_ui_dirty = true;
            radar_logf("RD03D: ack start node=%d", index);
            continue;
        }
        if (strcmp(message.text, "RADAR_ACK,STOP") == 0) {
            nodes[index].streaming = false;
            connect_ui_dirty = true;
            radar_logf("RD03D: ack stop node=%d", index);
            continue;
        }

        uint32_t sequence;
        uint8_t target_id;
        bool detected;
        int32_t x, y, speed, distance, angle;
        if (!parse_telemetry(message.text, &sequence, &target_id, &detected,
                             &x, &y, &speed, &distance, &angle)) {
            continue;
        }
        radar_target_t *target = &nodes[index].targets[target_id];
        uint32_t received_ms = now_ms();

        nodes[index].sequence = sequence;
        target->detected = detected;
        target->x_mm = x;
        target->y_mm = y;
        target->speed = speed;
        target->distance_mm = distance;
        target->angle_deg = angle;
        target->last_data_ms = received_ms;
        nodes[index].last_data_ms = received_ms;
        nodes[index].last_peer_ms = received_ms;
    }
}

static void send_command_to_selected(const char *command, bool streaming) {
    if (!api->espnow_send) return;
    for (int i = 0; i < node_count; ++i) {
        if (!nodes[i].used || !nodes[i].selected) continue;
        if (api->espnow_send(nodes[i].mac, command)) {
            nodes[i].streaming = streaming;
            connect_ui_dirty = true;
        }
    }
}

static void exit_app(void *user) {
    (void)user;
    GH_VOID(api, app_exit);
}

static void show_home_callback(void *user) {
    (void)user;
    radar_logf("RD03D: home callback from page=%d", page);
    show_home();
}

static void toggle_node(void *user) {
    int index = (int)(intptr_t)user;
    if (index < 0 || index >= node_count || !nodes[index].used) return;
    if (nodes[index].selected && nodes[index].streaming && api->espnow_send) {
        (void)api->espnow_send(nodes[index].mac, "RADAR_STOP");
        nodes[index].streaming = false;
    }
    nodes[index].selected = !nodes[index].selected;
    radar_logf("RD03D: toggle node=%d selected=%d", index,
               nodes[index].selected ? 1 : 0);
    update_node_button(index);
    update_connect_status();
}

static void connect_selected(void *user) {
    (void)user;
    if (!start_espnow()) return;
    radar_logf("RD03D: connect pressed selected=%d", selected_count());
    send_command_to_selected("RADAR_START", true);
    for (int i = 0; i < node_count; ++i) update_node_button(i);
    update_connect_status();
}

static void clear_selection(void *user) {
    (void)user;
    radar_logf("RD03D: clear pressed nodes=%d", node_count);
    if (api->espnow_send) {
        for (int i = 0; i < node_count; ++i) {
            if (nodes[i].used && nodes[i].streaming) {
                (void)api->espnow_send(nodes[i].mac, "RADAR_STOP");
            }
        }
    }
    for (int i = 0; i < node_count; ++i) {
        nodes[i].selected = false;
        nodes[i].streaming = false;
        update_node_button(i);
    }
    update_connect_status();
}

static void format_node_label(int index, char *out, size_t out_len) {
    int name_limit = layout.content_w >= 300 ? GHOSTESP_ESPNOW_NAME_MAX - 1 :
                     (layout.content_w >= 180 ? 16 : 8);
    snprintf(out, out_len, "%.*s", name_limit, nodes[index].name);
}

static void style_node_button(ghostesp_ui_obj_t button, bool selected) {
    if (!button) return;
    if (api->ui_obj_set_bg_color)
        api->ui_obj_set_bg_color(button, theme.surface_alt);
    if (api->ui_obj_set_border_color)
        api->ui_obj_set_border_color(button, selected ? theme.accent : theme.surface_alt);
    if (api->ui_obj_set_border_width)
        api->ui_obj_set_border_width(button, selected ? 2 : 1);
    if (api->ui_obj_set_radius) api->ui_obj_set_radius(button, 5);
}

static void update_node_check(int index) {
    if (index < 0 || index >= node_count || !nodes[index].used) return;
    ghostesp_ui_obj_t check = connect_checks[index];
    if (!check) return;

    bool connected = node_is_connected(&nodes[index]);
    if (!connect_check_state_valid[index] ||
        connect_check_state[index] != connected) {
        radar_logf("RD03D: check[%d] connected=%d streaming=%d peer_ms=%lu",
                   index, connected ? 1 : 0, nodes[index].streaming ? 1 : 0,
                   (unsigned long)nodes[index].last_peer_ms);
        connect_check_state[index] = connected;
        connect_check_state_valid[index] = true;
    } else {
        return;
    }
    if (connect_check_is_line[index]) {
        if (api->ui_obj_set_visible) api->ui_obj_set_visible(check, connected);
        return;
    }
    if (api->ui_label_set_text)
        api->ui_label_set_text(check, connected ? RADAR_CHECK_FALLBACK : " ");
    if (api->ui_obj_set_text_color)
        api->ui_obj_set_text_color(check,
                                   connected ? RADAR_CONNECTED_GREEN : theme.text_muted);
}

static void update_node_button(int index) {
    if (index < 0 || index >= node_count || !nodes[index].used) return;
    update_node_check(index);
    ghostesp_ui_obj_t button = connect_buttons[index];
    if (!button) return;
    char label[80];
    format_node_label(index, label, sizeof(label));
    if (api->ui_button_set_text) api->ui_button_set_text(button, label);
    style_node_button(button, nodes[index].selected);
}

static void add_node_row(ghostesp_ui_obj_t parent, int index) {
    if (index < 0 || index >= RADAR_MAX_NODES) return;
    bool active = index < node_count && nodes[index].used;

    char label[80];
    format_node_label(index, label, sizeof(label));

    ghostesp_ui_obj_t row = api->ui_card_create ? api->ui_card_create(parent) : NULL;
    if (!row) {
        connect_rows[index] = NULL;
        connect_buttons[index] = add_button(parent, label, toggle_node,
                                            (void *)(intptr_t)index);
        style_node_button(connect_buttons[index], active && nodes[index].selected);
        if (!active && connect_buttons[index] && api->ui_obj_set_visible)
            api->ui_obj_set_visible(connect_buttons[index], false);
        connect_row_visible[index] = active;
        return;
    }
    connect_rows[index] = row;
    connect_row_visible[index] = active;
    if (!active && api->ui_obj_set_visible)
        api->ui_obj_set_visible(row, false);
    if (api->ui_obj_set_bg_color) api->ui_obj_set_bg_color(row, theme.surface);
    if (api->ui_obj_set_border_color) api->ui_obj_set_border_color(row, theme.surface_alt);
    if (api->ui_obj_set_border_width) api->ui_obj_set_border_width(row, 1);
    if (api->ui_obj_set_radius) api->ui_obj_set_radius(row, 10);
    if (api->ui_obj_set_pad) api->ui_obj_set_pad(row, 5, 5, 2, 2);
    if (api->ui_obj_set_pad_row) api->ui_obj_set_pad_row(row, 0);
    if (api->ui_obj_set_scrollable) api->ui_obj_set_scrollable(row, false);
    if (api->ui_obj_set_flex_flow)
        api->ui_obj_set_flex_flow(row, GHOSTESP_FLEX_FLOW_ROW);
    if (api->ui_obj_set_pad_column) api->ui_obj_set_pad_column(row, 4);

    bool connected = active && node_is_connected(&nodes[index]);
    ghostesp_ui_obj_t check_slot = api->ui_card_create ? api->ui_card_create(row) : NULL;
    if (check_slot) {
        style_flat_container(check_slot, GHOSTESP_FLEX_FLOW_ROW);
        if (api->ui_obj_set_bg_color) api->ui_obj_set_bg_color(check_slot, theme.surface);
        if (api->ui_obj_set_width) api->ui_obj_set_width(check_slot, 28);
        if (api->ui_obj_set_height) api->ui_obj_set_height(check_slot, 28);
    }
    ghostesp_ui_obj_t check = NULL;
    if (check_slot && api->ui_line_create && api->ui_line_set_points) {
        check = api->ui_line_create(check_slot);
        if (check) {
            ghostesp_point_t check_points[3] = {{3, 12}, {9, 18}, {21, 5}};
            api->ui_line_set_points(check, check_points, 3);
            if (api->ui_line_set_color)
                api->ui_line_set_color(check, RADAR_CONNECTED_GREEN);
            if (api->ui_line_set_width) api->ui_line_set_width(check, 3);
            if (api->ui_obj_set_size) api->ui_obj_set_size(check, 24, 24);
            if (api->ui_obj_set_visible) api->ui_obj_set_visible(check, connected);
            connect_check_is_line[index] = true;
        }
    }
    if (!check) {
        check = add_label(check_slot ? check_slot : row,
                          connected ? RADAR_CHECK_FALLBACK : " ",
                          GHOSTESP_FONT_BODY,
                          connected ? RADAR_CONNECTED_GREEN : theme.text_muted);
        if (check && api->ui_obj_set_width) api->ui_obj_set_width(check, 28);
        connect_check_is_line[index] = false;
    }
    connect_checks[index] = check;
    connect_check_state[index] = connected;
    connect_check_state_valid[index] = true;

    ghostesp_ui_obj_t button = add_button(row, label, toggle_node,
                                          (void *)(intptr_t)index);
    connect_buttons[index] = button;
    if (button && api->ui_obj_set_width) {
        int32_t button_width = layout.content_w < 180 ? layout.content_w - 44
                                                       : layout.content_w - 52;
        if (button_width < 32) button_width = 32;
        api->ui_obj_set_width(button, button_width);
        if (api->ui_obj_set_height) {
            api->ui_obj_set_height(button, layout.compact ? 28 : 30);
            api->ui_obj_set_height(row, layout.compact ? 32 : 34);
        }
        style_node_button(button, active && nodes[index].selected);
    }
}

static void refresh_connect_rows(void) {
    if (!connect_list && !connect_screen) return;

    for (int i = 0; i < RADAR_MAX_NODES; ++i) {
        bool active = i < node_count && nodes[i].used;
        if (connect_rows[i] && connect_row_visible[i] != active &&
            api->ui_obj_set_visible) {
            api->ui_obj_set_visible(connect_rows[i], active);
            connect_row_visible[i] = active;
        }
        if (connect_buttons[i] && !connect_rows[i] &&
            api->ui_obj_set_visible) {
            api->ui_obj_set_visible(connect_buttons[i], active);
        }
        if (active) update_node_button(i);
    }
    for (int i = connect_rendered_count; i < node_count; ++i)
        add_node_row(connect_list, i);
    if (node_count > connect_rendered_count)
        connect_rendered_count = node_count;
    if (connect_empty_label && api->ui_obj_set_visible &&
        connect_empty_visible != (node_count == 0)) {
        connect_empty_visible = node_count == 0;
        api->ui_obj_set_visible(connect_empty_label, connect_empty_visible);
    }
    update_connect_status();
}

static void show_connect(void) {
    show_connect_page();
}

static void show_connect_page(void) {
    bool was_active = api->espnow_is_active && api->espnow_is_active();
    if (!start_espnow()) {
        show_home();
        return;
    }
    if (was_active && api->espnow_announce) {
        api->espnow_announce();
        last_announce_ms = now_ms();
    }
    page = PAGE_CONNECT;
    refresh_nodes();
    if (connect_screen) {
        radar_logf("RD03D: show connect cached nodes=%d", node_count);
        activate_screen(connect_screen);
        refresh_connect_rows();
        return;
    }
    radar_logf("RD03D: show connect create nodes=%d", node_count);
    activate_screen(NULL);
    connect_screen = create_page_screen();
    screen = connect_screen;
    if (!screen) return;
    activate_screen(screen);
    if (api->ui_obj_set_flex_flow)
        api->ui_obj_set_flex_flow(screen, GHOSTESP_FLEX_FLOW_COLUMN);
    bool narrow = layout.content_w < 180;
    int32_t outer_pad = layout.compact ? 4 : 8;
    int32_t row_gap = layout.compact ? 2 : 6;
    int32_t status_height = narrow ? 14 : 16;
    int32_t action_height = narrow ? (layout.compact ? 24 : 28)
                                  : (layout.compact ? 28 : 34);
    bool action_row = narrow && api->ui_card_create;
    int32_t actions_height = action_row ? action_height
                                        : action_height * 3 + row_gap * 2;
    if (api->ui_obj_set_pad)
        api->ui_obj_set_pad(screen, outer_pad, outer_pad, outer_pad, outer_pad);
    if (api->ui_obj_set_pad_row) api->ui_obj_set_pad_row(screen, row_gap);
    if (api->ui_obj_set_scrollable) api->ui_obj_set_scrollable(screen, false);

    status_label = add_label(screen, "", GHOSTESP_FONT_MICRO, theme.text_muted);
    if (status_label) {
        if (api->ui_obj_set_height) api->ui_obj_set_height(status_label, status_height);
        if (api->ui_obj_set_width)
            api->ui_obj_set_width(status_label, layout.content_w - outer_pad * 2);
    }
    update_connect_status();

    int32_t list_height = layout.content_h - outer_pad * 2 - status_height -
                          actions_height - row_gap * 2;
    if (list_height < 28) list_height = 28;
    connect_list = api->ui_card_create ? api->ui_card_create(screen) : NULL;
    ghostesp_ui_obj_t list_parent = connect_list ? connect_list : screen;
    if (connect_list) {
        style_flat_container(connect_list, GHOSTESP_FLEX_FLOW_COLUMN);
        if (api->ui_obj_set_pad_row) api->ui_obj_set_pad_row(connect_list, 2);
        if (api->ui_obj_set_height) api->ui_obj_set_height(connect_list, list_height);
        if (api->ui_obj_set_scrollable) api->ui_obj_set_scrollable(connect_list, true);
        if (api->ui_obj_set_scrollbar) api->ui_obj_set_scrollbar(connect_list, true);
    }

    connect_empty_label = add_label(list_parent, "Waiting for RD03D radars...",
                                    GHOSTESP_FONT_BODY, theme.text);
    connect_empty_visible = node_count == 0;
    if (connect_empty_label && api->ui_obj_set_visible && !connect_empty_visible)
        api->ui_obj_set_visible(connect_empty_label, false);
    for (int i = 0; i < RADAR_CONNECT_ROW_POOL; ++i) {
        add_node_row(list_parent, i);
    }
    connect_rendered_count = RADAR_CONNECT_ROW_POOL;

    if (action_row) {
        ghostesp_ui_obj_t actions = api->ui_card_create(screen);
        if (actions) {
            style_flat_container(actions, GHOSTESP_FLEX_FLOW_ROW);
            if (api->ui_obj_set_height) api->ui_obj_set_height(actions, action_height);
            if (api->ui_obj_set_pad_column) api->ui_obj_set_pad_column(actions, row_gap);
            if (api->ui_obj_set_flex_align)
                api->ui_obj_set_flex_align(actions, GHOSTESP_FLEX_ALIGN_SPACE_BETWEEN,
                                           GHOSTESP_FLEX_ALIGN_CENTER,
                                           GHOSTESP_FLEX_ALIGN_CENTER);

            int32_t action_width = (layout.content_w - outer_pad * 2 - row_gap * 2) / 3;
            if (action_width < 20) action_width = 20;
            const char *connect_text = layout.content_w < 120 ? "Go" : "Connect";
            ghostesp_ui_obj_t button = add_button(actions, connect_text,
                                                  connect_selected, NULL);
            if (button) {
                if (api->ui_obj_set_width) api->ui_obj_set_width(button, action_width);
                if (api->ui_obj_set_height) api->ui_obj_set_height(button, action_height);
            }
            button = add_button(actions, "Clear", clear_selection, NULL);
            if (button) {
                if (api->ui_obj_set_width) api->ui_obj_set_width(button, action_width);
                if (api->ui_obj_set_height) api->ui_obj_set_height(button, action_height);
            }
            button = add_button(actions, "Back", show_home_callback, NULL);
            if (button) {
                if (api->ui_obj_set_width) api->ui_obj_set_width(button, action_width);
                if (api->ui_obj_set_height) api->ui_obj_set_height(button, action_height);
            }
        }
    } else {
        ghostesp_ui_obj_t button = add_button(screen, "Connect", connect_selected, NULL);
        if (button && api->ui_obj_set_height) api->ui_obj_set_height(button, action_height);
        button = add_button(screen, "Clear selection", clear_selection, NULL);
        if (button && api->ui_obj_set_height) api->ui_obj_set_height(button, action_height);
        button = add_button(screen, "Back", show_home_callback, NULL);
        if (button && api->ui_obj_set_height) api->ui_obj_set_height(button, action_height);
    }
}

static const int16_t radar_sweep_sin_milli[RADAR_HALF_ANGLE_DEG + 1] = {
    0, 17, 35, 52, 70, 87, 105, 122, 139, 156, 174,
    191, 208, 225, 242, 259, 276, 292, 309, 326, 342,
    358, 375, 391, 407, 423, 438, 454, 469, 485, 500,
    515, 530, 545, 559, 574, 588, 602, 616, 629, 643,
};

static const int16_t radar_sweep_cos_milli[RADAR_HALF_ANGLE_DEG + 1] = {
    1000, 1000, 999, 999, 998, 996, 995, 993, 990, 988, 985,
    982, 978, 974, 970, 966, 961, 956, 951, 946, 940,
    934, 927, 921, 914, 906, 899, 891, 883, 875, 866,
    857, 848, 839, 829, 819, 809, 799, 788, 777, 766,
};

static void get_radar_geometry(int *cx, int *cy, int *radius) {
    int width = radar_canvas_width;
    int height = radar_canvas_height;
    *cx = width / 2;
    *cy = height - 2;

    /* Size the 80-degree cone from its actual footprint and use
     * nearly all of the available vertical canvas. */
    int by_height = height - 4;
    int by_width = width * 1000 / (RADAR_CONE_SIN_MILLI * 2) - 2;
    *radius = by_height < by_width ? by_height : by_width;
    if (*radius < 20) *radius = 20;
}

static void position_range_labels(void) {
    if (!api->ui_obj_set_pos) return;
    int cx, cy, radius;
    get_radar_geometry(&cx, &cy, &radius);
    for (int i = 0; i < RADAR_RANGE_RINGS; ++i) {
        if (!range_labels[i]) continue;
        int ring_radius = radius * (i + 1) / RADAR_RANGE_RINGS;
        int label_width = 24;
        int label_offset = i == RADAR_RANGE_RINGS - 2 ? 18 : 10;
        int x = cx + ring_radius * RADAR_CONE_SIN_MILLI / 1000 + label_offset;
        if (i == RADAR_RANGE_RINGS - 2)
            x = radar_canvas_width - label_width - 2;
        int y = cy - ring_radius * RADAR_CONE_COS_MILLI / 1000 - 7;
        int max_x = radar_canvas_width - label_width - 2;
        int max_y = radar_canvas_height - 14;
        if (max_x < 0) max_x = 0;
        if (max_y < 0) max_y = 0;
        if (x < 0) x = 0;
        if (x > max_x) x = max_x;
        if (y < 0) y = 0;
        if (y > max_y) y = max_y;
        api->ui_obj_set_pos(range_labels[i], x, y);
    }
}

static void draw_radar_background(void) {
    if (!canvas || !api->ui_canvas_fill) return;

    int32_t width = api->ui_obj_get_width ? api->ui_obj_get_width(canvas) : 220;
    int32_t height = api->ui_obj_get_height ? api->ui_obj_get_height(canvas) : 180;
    if (width < 16) width = 16;
    if (height < 16) height = 16;
    if (radar_background_drawn && width == radar_canvas_width &&
        height == radar_canvas_height) return;

    radar_canvas_width = width;
    radar_canvas_height = height;
    radar_background_drawn = true;

    api->ui_canvas_fill(canvas, theme.bg);
    int cx, cy, radius;
    get_radar_geometry(&cx, &cy, &radius);

    if (api->ui_canvas_draw_arc) {
        for (int ring = 1; ring <= RADAR_RANGE_RINGS; ++ring) {
            api->ui_canvas_draw_arc(canvas, cx, cy,
                                    radius * ring / RADAR_RANGE_RINGS,
                                    270 - RADAR_HALF_ANGLE_DEG,
                                    270 + RADAR_HALF_ANGLE_DEG,
                                    RADAR_GRID_GREEN, 1);
        }
    }
    if (api->ui_canvas_draw_line) {
        ghostesp_point_t left[2] = {
            {cx, cy},
            {cx - radius * RADAR_CONE_SIN_MILLI / 1000,
             cy - radius * RADAR_CONE_COS_MILLI / 1000},
        };
        ghostesp_point_t right[2] = {
            {cx, cy},
            {cx + radius * RADAR_CONE_SIN_MILLI / 1000,
             cy - radius * RADAR_CONE_COS_MILLI / 1000},
        };
        ghostesp_point_t center[2] = {{cx, cy}, {cx, cy - radius}};
        api->ui_canvas_draw_line(canvas, left, 2, RADAR_CONE_GREEN, 3);
        api->ui_canvas_draw_line(canvas, right, 2, RADAR_CONE_GREEN, 3);
        api->ui_canvas_draw_line(canvas, center, 2, RADAR_GRID_GREEN, 1);
    }
    if (api->ui_canvas_draw_rect) {
        api->ui_canvas_draw_rect(canvas, cx - 3, cy - 3, 7, 7, RADAR_CONE_GREEN);
    }
    position_range_labels();
}

static void update_sweep_line(void) {
    if (!sweep_line || !api->ui_line_set_points) return;
    if (sweep_drawn_angle_deg == sweep_angle_deg) return;

    int cx, cy, radius;
    get_radar_geometry(&cx, &cy, &radius);
    int angle = sweep_angle_deg < 0 ? -sweep_angle_deg : sweep_angle_deg;
    if (angle > RADAR_HALF_ANGLE_DEG) angle = RADAR_HALF_ANGLE_DEG;
    int direction = sweep_angle_deg < 0 ? -1 : 1;
    int endpoint_x = cx + direction * radius * radar_sweep_sin_milli[angle] / 1000;
    int endpoint_y = cy - radius * radar_sweep_cos_milli[angle] / 1000;
    ghostesp_point_t points[2] = {{cx, cy}, {endpoint_x, endpoint_y}};
    api->ui_line_set_points(sweep_line, points, 2);
    if (api->ui_obj_set_visible) api->ui_obj_set_visible(sweep_line, true);
    sweep_drawn_angle_deg = sweep_angle_deg;
}

static void advance_sweep(uint32_t elapsed_ms) {
    sweep_elapsed_ms += elapsed_ms;
    while (sweep_elapsed_ms >= RADAR_SWEEP_STEP_MS) {
        sweep_elapsed_ms -= RADAR_SWEEP_STEP_MS;
        sweep_angle_deg += sweep_direction * RADAR_SWEEP_STEP_DEG;
        if (sweep_angle_deg >= RADAR_HALF_ANGLE_DEG) {
            sweep_angle_deg = RADAR_HALF_ANGLE_DEG;
            sweep_direction = -1;
        } else if (sweep_angle_deg <= -RADAR_HALF_ANGLE_DEG) {
            sweep_angle_deg = -RADAR_HALF_ANGLE_DEG;
            sweep_direction = 1;
        }
    }
}

static void update_radar_marker(void) {
    if (!canvas || !api->ui_line_set_points) return;
    draw_radar_background();

    int index = selected_node_at(view_slot);
    uint32_t now = now_ms();
    int cx = 0;
    int cy = 0;
    int radius = 0;
    get_radar_geometry(&cx, &cy, &radius);

    for (int target_id = 0; target_id < RADAR_MAX_TARGETS; ++target_id) {
        ghostesp_ui_obj_t marker = target_markers[target_id];
        bool visible = false;
        int px = 0;
        int py = 0;

        if (index >= 0) {
            radar_target_t *target = &nodes[index].targets[target_id];
            bool fresh = target->last_data_ms &&
                         now - target->last_data_ms <= RADAR_DATA_TIMEOUT_MS;
            if (fresh && target->detected) {
                int width = radar_canvas_width;
                int height = radar_canvas_height;
                px = cx + (target->x_mm * radius / RADAR_MAX_RANGE_MM);
                py = cy - (target->y_mm * radius / RADAR_MAX_RANGE_MM);
                if (px < 5) px = 5;
                if (px > width - 6) px = width - 6;
                if (py < 5) py = 5;
                if (py > height - 6) py = height - 6;
                visible = true;
            }
        }

        if (!visible || !marker) {
            if (marker_visible[target_id] && marker && api->ui_obj_set_visible)
                api->ui_obj_set_visible(marker, false);
            marker_visible[target_id] = false;
            continue;
        }

        if (!marker_visible[target_id] || marker_x[target_id] != px ||
            marker_y[target_id] != py) {
            ghostesp_point_t points[5] = {
                {px - 4, py - 4},
                {px + 4, py - 4},
                {px + 4, py + 4},
                {px - 4, py + 4},
                {px - 4, py - 4},
            };
            api->ui_line_set_points(marker, points, 5);
            marker_x[target_id] = px;
            marker_y[target_id] = py;
        }
        if (!marker_visible[target_id] && api->ui_obj_set_visible)
            api->ui_obj_set_visible(marker, true);
        marker_visible[target_id] = true;
    }
}

static void draw_radar(void) {
    draw_radar_background();
    update_sweep_line();
    update_radar_marker();
}

static void format_meters(char *out, size_t out_len, int32_t millimeters) {
    int32_t value = millimeters;
    bool negative = value < 0;
    if (negative) value = -value;
    snprintf(out, out_len, "%s%ld.%02ldm", negative ? "-" : "",
             (long)(value / 1000), (long)((value % 1000) / 10));
}

static void update_radar_labels(void) {
    int total = selected_count();
    bool narrow = layout.content_w < 180;
    if (total == 0) {
        if (radar_label && api->ui_label_set_text)
            api->ui_label_set_text(radar_label, "No radar selected");
        if (radar_data_label && api->ui_label_set_text)
            api->ui_label_set_text(radar_data_label,
                                   narrow ? "Use Connect" : "Use Connect to select a radar");
        return;
    }

    if (view_slot >= total) view_slot = 0;
    int index = selected_node_at(view_slot);
    if (index < 0) return;
    radar_node_t *node = &nodes[index];

    char header[64];
    char data[256];
    uint32_t now = now_ms();
    int detected_count = 0;
    for (int target_id = 0; target_id < RADAR_MAX_TARGETS; ++target_id) {
        radar_target_t *target = &node->targets[target_id];
        if (target->detected && target->last_data_ms &&
            now - target->last_data_ms <= RADAR_DATA_TIMEOUT_MS) {
            ++detected_count;
        }
    }

    if (detected_count == 0) {
        if (radar_label && api->ui_label_set_text) {
            snprintf(header, sizeof(header), narrow ? "No target %d/%d" :
                     "No target  |  Radar %d/%d", view_slot + 1, total);
        }
        snprintf(data, sizeof(data), narrow ? "2m 4m 6m 8m | 80deg" :
                 "Range 2m  4m  6m  8m  |  FOV 80 deg");
    } else {
        if (radar_label && api->ui_label_set_text)
            snprintf(header, sizeof(header), narrow ? "%d targets %d/%d" :
                     "%d targets  |  Radar %d/%d", detected_count,
                     view_slot + 1, total);

        size_t used = 0;
        for (int target_id = 0; target_id < RADAR_MAX_TARGETS; ++target_id) {
            radar_target_t *target = &node->targets[target_id];
            if (!target->detected || !target->last_data_ms ||
                now - target->last_data_ms > RADAR_DATA_TIMEOUT_MS) {
                continue;
            }
            if (used >= sizeof(data)) break;
            char distance[20];
            char x[20];
            char y[20];
            format_meters(distance, sizeof(distance), target->distance_mm);
            format_meters(x, sizeof(x), target->x_mm);
            format_meters(y, sizeof(y), target->y_mm);
            int written = snprintf(
                data + used, sizeof(data) - used,
                narrow ? "Target%d S:%ld D:%s A:%ld\n" :
                         "Target%d S:%ld D:%s A:%ld X:%s Y:%s\n",
                target_id + 1, (long)target->speed, distance,
                (long)target->angle_deg, x, y);
            if (written < 0) break;
            if ((size_t)written >= sizeof(data) - used) {
                used = sizeof(data) - 1;
                break;
            }
            used += (size_t)written;
        }
        if (used > 0 && used < sizeof(data) && data[used - 1] == '\n')
            data[used - 1] = '\0';
    }
    if (radar_label && api->ui_label_set_text)
        api->ui_label_set_text(radar_label, header);
    if (radar_data_label && api->ui_label_set_text)
        api->ui_label_set_text(radar_data_label, data);
}

static void previous_radar(void *user) {
    (void)user;
    int total = selected_count();
    if (total > 0) view_slot = (view_slot + total - 1) % total;
    update_radar_labels();
    draw_radar();
}

static void next_radar(void *user) {
    (void)user;
    int total = selected_count();
    if (total > 0) view_slot = (view_slot + 1) % total;
    update_radar_labels();
    draw_radar();
}

static void show_radar(void) {
    page = PAGE_RADAR;
    refresh_nodes();
    sweep_angle_deg = 0;
    sweep_direction = 1;
    sweep_elapsed_ms = 0;
    sweep_drawn_angle_deg = -1;
    if (radar_screen) {
        radar_logf("RD03D: show radar cached selected=%d", selected_count());
        activate_screen(radar_screen);
        update_radar_labels();
        draw_radar();
        return;
    }
    radar_logf("RD03D: show radar create selected=%d", selected_count());
    activate_screen(NULL);
    radar_screen = create_page_screen();
    screen = radar_screen;
    if (!screen) return;
    activate_screen(screen);

    bool narrow = layout.content_w < 180;
    bool tiny = layout.content_w < 100;
    int32_t margin = narrow ? 2 : 4;
    int32_t gap = narrow ? 2 : 4;
    int32_t width = layout.content_w - margin * 2;
    if (width < 40) width = 40;
    int32_t header_height = tiny ? 14 : 16;
    int32_t data_height = tiny ? 14 : 48;
    int32_t nav_button_height = narrow ? (layout.compact ? 22 : 26) : 30;
    int32_t nav_height = narrow ? nav_button_height * 2 + gap : nav_button_height;
    int32_t canvas_y = header_height + gap + data_height + gap;
    int32_t canvas_height = layout.content_h - canvas_y - nav_height - gap - margin;
    if (canvas_height < 32 && !tiny) {
        header_height = 14;
        canvas_y = header_height + gap + data_height + gap;
        canvas_height = layout.content_h - canvas_y - nav_height - gap - margin;
    }
    if (canvas_height < 24) canvas_height = 24;

    /* Keep the page in a measured column so the fixed canvas and navigation
     * remain inside the 320x240 content area. */
    if (api->ui_obj_set_flex_flow)
        api->ui_obj_set_flex_flow(screen, GHOSTESP_FLEX_FLOW_COLUMN);
    if (api->ui_obj_set_pad)
        api->ui_obj_set_pad(screen, margin, margin, 0, margin);
    if (api->ui_obj_set_pad_row) api->ui_obj_set_pad_row(screen, gap);
    if (api->ui_obj_set_scrollable) api->ui_obj_set_scrollable(screen, false);

    radar_label = add_label(screen, "No target", GHOSTESP_FONT_MICRO, theme.text_muted);
    if (radar_label) {
        if (api->ui_obj_set_size)
            api->ui_obj_set_size(radar_label, width, header_height);
    }
    radar_data_label = add_label(screen, "", GHOSTESP_FONT_MICRO, theme.text_muted);
    if (radar_data_label && api->ui_obj_set_size)
        api->ui_obj_set_size(radar_data_label, width, data_height);

    if (api->ui_canvas_create) {
        canvas = api->ui_canvas_create(screen, width, canvas_height);
        if (canvas && api->ui_obj_set_size)
            api->ui_obj_set_size(canvas, width, canvas_height);
        if (canvas && api->ui_line_create) {
            sweep_line = api->ui_line_create(canvas);
            if (sweep_line) {
                if (api->ui_line_set_color)
                    api->ui_line_set_color(sweep_line, RADAR_CONE_GREEN);
                if (api->ui_line_set_width)
                    api->ui_line_set_width(sweep_line, 1);
                if (api->ui_obj_set_visible)
                    api->ui_obj_set_visible(sweep_line, false);
            }
            for (int target_id = 0; target_id < RADAR_MAX_TARGETS; ++target_id) {
                target_markers[target_id] = api->ui_line_create(canvas);
                if (target_markers[target_id]) {
                    if (api->ui_line_set_color)
                        api->ui_line_set_color(target_markers[target_id],
                                               RADAR_TARGET_RED);
                    if (api->ui_line_set_width)
                        api->ui_line_set_width(target_markers[target_id], 2);
                    if (api->ui_obj_set_visible)
                        api->ui_obj_set_visible(target_markers[target_id], false);
                }
            }
        }
        if (canvas && api->ui_label_create) {
            static const char *range_texts[RADAR_RANGE_RINGS] = {
                "2m", "4m", "6m", "8m",
            };
            for (int i = 0; i < RADAR_RANGE_RINGS; ++i) {
                range_labels[i] = add_label(canvas, range_texts[i],
                                            GHOSTESP_FONT_MICRO, RADAR_CONE_GREEN);
                if (range_labels[i] && api->ui_obj_set_width)
                    api->ui_obj_set_width(range_labels[i], 24);
            }
        }
    }
    update_radar_labels();
    draw_radar();

    ghostesp_ui_obj_t nav = api->ui_card_create ? api->ui_card_create(screen) : screen;
    if (nav != screen) {
        style_flat_container(nav, narrow ? GHOSTESP_FLEX_FLOW_COLUMN :
                                      GHOSTESP_FLEX_FLOW_ROW);
        if (api->ui_obj_set_size)
            api->ui_obj_set_size(nav, width, nav_height);
        if (api->ui_obj_set_flex_align)
            api->ui_obj_set_flex_align(nav, narrow ? GHOSTESP_FLEX_ALIGN_START :
                                                GHOSTESP_FLEX_ALIGN_SPACE_BETWEEN,
                                       GHOSTESP_FLEX_ALIGN_CENTER,
                                       GHOSTESP_FLEX_ALIGN_CENTER);
        if (narrow) {
            if (api->ui_obj_set_pad_row) api->ui_obj_set_pad_row(nav, gap);
        } else if (api->ui_obj_set_pad_column) {
            api->ui_obj_set_pad_column(nav, gap);
        }
    }

    int32_t button_width = narrow ? width : (width - gap) / 2;
    ghostesp_ui_obj_t previous = add_radar_button(nav, "Previous", previous_radar,
                                                  button_width);
    ghostesp_ui_obj_t next = add_radar_button(nav, "Next", next_radar, button_width);
    if (previous && api->ui_obj_set_height)
        api->ui_obj_set_height(previous, nav_button_height);
    if (next && api->ui_obj_set_height)
        api->ui_obj_set_height(next, nav_button_height);
}

static void open_connect(void *user) {
    (void)user;
    show_connect();
}

static void open_radar(void *user) {
    (void)user;
    if (selected_count() == 0) {
        if (api->toast) api->toast("Select a radar from Connect first");
        show_connect();
        return;
    }
    show_radar();
}

static void show_home(void) {
    page = PAGE_HOME;
    if (home_screen) {
        radar_logf("RD03D: show home cached");
        activate_screen(home_screen);
        update_home_status();
        return;
    }
    radar_logf("RD03D: show home create");
    activate_screen(NULL);
    home_screen = create_page_screen();
    screen = home_screen;
    if (!screen) return;
    activate_screen(screen);
    if (api->ui_obj_set_flex_flow)
        api->ui_obj_set_flex_flow(screen, GHOSTESP_FLEX_FLOW_COLUMN);
    if (api->ui_obj_set_pad) api->ui_obj_set_pad(screen, 10, 10, 10, 10);

    home_status_label = add_label(screen, "", GHOSTESP_FONT_BODY, theme.text_muted);
    update_home_status();
    add_button(screen, "Connect", open_connect, NULL);
    add_button(screen, "Radar", open_radar, NULL);
    add_button(screen, "Exit", exit_app, NULL);
}

static void radar_start(void) {
    memset(nodes, 0, sizeof(nodes));
    node_count = 0;
    view_slot = 0;
    last_announce_ms = 0;
    gh_theme_init(api, &theme);
    gh_layout_init(api, &layout);
    app_root = api->ui_screen_create ? api->ui_screen_create("RD-03D Radar") : NULL;
    if (!app_root) {
        radar_logf("RD03D: app root create failed");
        return;
    }
    if (api->ui_obj_set_pad) api->ui_obj_set_pad(app_root, 0, 0, 0, 0);
    if (api->ui_obj_set_bg_color) api->ui_obj_set_bg_color(app_root, theme.bg);
    radar_logf("RD03D: app root created");
    radar_logf("RD03D: entering home setup");
    show_home();
}

static void radar_stop(void) {
    if (api->espnow_send) {
        for (int i = 0; i < node_count; ++i) {
            if (nodes[i].used && nodes[i].streaming)
                (void)api->espnow_send(nodes[i].mac, "RADAR_STOP");
        }
    }
    if (api->espnow_stop && api->espnow_is_active && api->espnow_is_active())
        api->espnow_stop();
    delete_screen();
    radar_logf("RD03D: app stopped and pages deleted");
    if (app_root && api->ui_obj_delete) api->ui_obj_delete(app_root);
    app_root = NULL;
}

static void radar_input(const ghostesp_input_event_t *event) {
    if (!event || !event->pressed) return;
    radar_logf("RD03D: input page=%d type=%d", page, event->type);
    if ((page == PAGE_RADAR || page == PAGE_CONNECT) &&
        event->type == GHOSTESP_INPUT_LEFT) {
        show_home();
        return;
    }
    if (event->type == GHOSTESP_INPUT_BACK) {
        if (page == PAGE_HOME) exit_app(NULL);
        else show_home();
    }
}

static void radar_tick(uint32_t elapsed_ms) {
    drain_messages();
    uint32_t now = now_ms();
    if (page == PAGE_CONNECT) {
        bool changed = refresh_nodes();
        if (changed || connect_ui_dirty) {
            if (changed) radar_logf("RD03D: scan changed nodes=%d", node_count);
            refresh_connect_rows();
            connect_ui_dirty = false;
        }
    } else {
        refresh_nodes();
    }
    if (api->espnow_is_active && api->espnow_is_active() &&
        (last_announce_ms == 0 || now - last_announce_ms >= RADAR_HELLO_INTERVAL_MS)) {
        radar_logf("RD03D: scan announce page=%d", page);
        if (api->espnow_announce) api->espnow_announce();
        last_announce_ms = now;
    }
    if (page == PAGE_RADAR) {
        advance_sweep(elapsed_ms);
        update_radar_labels();
        draw_radar();
    }
}

static const ghostesp_app_t app = GHOSTESP_APP_DEFINE(
    "rd03d_radar", "RD-03D Radar", radar_start, radar_stop, radar_input, radar_tick);

GHOSTESP_APP_INIT_WITH_API(app, api, "rd03d_radar", GHOSTESP_API_STRUCT_SIZE_V1)
