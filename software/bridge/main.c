#include <arpa/inet.h>
#include <cjson/cJSON.h>
#include <cmp/cmp.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <zmq.h>

typedef struct {
    uint8_t *buffer;
    size_t capacity;
    size_t size;
    size_t position;
} buffer_t;

static bool buffer_reader(cmp_ctx_t *ctx, void *data, size_t count) {
    buffer_t *buf = (buffer_t *)ctx->buf;

    if((buf->position + count) > buf->capacity) {
        return false;
    }

    memcpy(data, buf->buffer + buf->position, count);
    buf->position += count;

    return true;
}

static size_t buffer_writer(cmp_ctx_t *ctx, const void *data, size_t count) {
    buffer_t *buf = (buffer_t *)ctx->buf;

    if((buf->position + count) > buf->capacity) {
        return 0;
    }

    memcpy(buf->buffer + buf->position, data, count);
    buf->position += count;
    buf->size = buf->position;

    return count;
}

static void json_to_msgpack(cmp_ctx_t *cmp, const cJSON *json) {
    if(cJSON_IsObject(json)) {
        const int size = cJSON_GetArraySize(json);
        cmp_write_map(cmp, size);

        cJSON *child = NULL;
        cJSON_ArrayForEach(child, json) {
            cmp_write_str(cmp, child->string, strlen(child->string));
            json_to_msgpack(cmp, child);
        }

        return;
    }

    if(cJSON_IsArray(json)) {
        const int size = cJSON_GetArraySize(json);
        cmp_write_array(cmp, size);

        cJSON *element = NULL;
        cJSON_ArrayForEach(element, json) {
            json_to_msgpack(cmp, element);
        }

        return;
    }

    if(cJSON_IsString(json)) {
        const char *str = cJSON_GetStringValue(json);
        cmp_write_str(cmp, str, strlen(str));
        return;
    }

    if(cJSON_IsNumber(json)) {
        cmp_write_float(cmp, json->valuedouble);
        return;
    }

    if(cJSON_IsBool(json)) {
        cmp_write_bool(cmp, cJSON_IsTrue(json));
        return;
    }

    if(cJSON_IsNull(json)) {
        cmp_write_nil(cmp);
        return;
    }
}

static cJSON *msgpack_to_json(cmp_ctx_t *cmp) {
    cmp_object_t object;
    if(!cmp_read_object(cmp, &object)) {
        return NULL;
    }

    uint32_t array_size = 0;
    if(cmp_object_as_array(&object, &array_size)) {
        cJSON *array = cJSON_CreateArray();

        for(uint32_t i = 0; i < array_size; i++) {
            cJSON_AddItemToArray(array, msgpack_to_json(cmp));
        }

        return array;
    }

    uint32_t map_size = 0;
    if(cmp_object_as_map(&object, &map_size)) {
        cJSON *object = cJSON_CreateObject();

        for(uint32_t i = 0; i < map_size; i++) {
            char key[256];
            uint32_t key_len = sizeof(key);
            if(!cmp_read_str(cmp, key, &key_len)) {
                return NULL;
            }

            cJSON_AddItemToObject(object, key, msgpack_to_json(cmp));
        }

        return object;
    }

    char str[256];
    uint32_t str_len = 0;
    if(cmp_object_as_str(&object, &str_len)) {
        if(!cmp_object_to_str(cmp, &object, str, sizeof(str))) {
            return NULL;
        }
        return cJSON_CreateString(str);
    }

    int64_t integer = 0;
    if(cmp_object_as_long(&object, &integer)) {
        return cJSON_CreateNumber(integer);
    }

    float floating = 0;
    if(cmp_object_as_float(&object, &floating)) {
        return cJSON_CreateNumber(floating);
    }

    bool boolean = false;
    if(cmp_object_as_bool(&object, &boolean)) {
        return cJSON_CreateBool(boolean);
    }

    return cJSON_CreateNull();
}

static void *udp_receiver(void *args) {
    (void)args;

    const int sock = socket(AF_INET, SOCK_DGRAM, 0);

    struct sockaddr_in server_addr = {0};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(4444);
    bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));

    void *context = zmq_ctx_new();
    void *publisher = zmq_socket(context, ZMQ_PUB);
    zmq_bind(publisher, "tcp://localhost:6000");

    char buffer_rx[65536];
    struct sockaddr_in addr;
    socklen_t addr_len = sizeof(addr);

    while(1) {
        const ssize_t size =
            recvfrom(sock, buffer_rx, sizeof(buffer_rx), 0, (struct sockaddr *)&addr, &addr_len);

        buffer_t buffer = {
            .buffer = (uint8_t *)buffer_rx,
            .capacity = size,
            .size = size,
            .position = 0,
        };
        cmp_ctx_t cmp;
        cmp_init(&cmp, &buffer, buffer_reader, NULL, NULL);

        cJSON *json = msgpack_to_json(&cmp);
        if(!json) {
            continue;
        }

        char *str = cJSON_Print(json);

        zmq_send(publisher, str, strlen(str), 0);

        free(str);
        cJSON_Delete(json);
    }

    zmq_close(publisher);
    close(sock);

    return NULL;
}

static void *udp_transmitter(void *args) {
    (void)args;

    const int sock = socket(AF_INET, SOCK_DGRAM, 0);

    struct sockaddr_in send_addr = {0};
    send_addr.sin_family = AF_INET;
    send_addr.sin_addr.s_addr = inet_addr("10.42.0.19");
    send_addr.sin_port = htons(3333);

    void *context = zmq_ctx_new();
    void *subscriber = zmq_socket(context, ZMQ_SUB);
    zmq_bind(subscriber, "tcp://localhost:7000");
    zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0);

    char buffer_rx[65536];
    char buffer_tx[65536];

    while(1) {
        const int size = zmq_recv(subscriber, buffer_rx, sizeof(buffer_rx), 0);
        if(size <= 0) {
            continue;
        }

        printf("%.*s\n\r", size, buffer_rx);

        cJSON *json = cJSON_Parse(buffer_rx);
        if(!json) {
            continue;
        }

        buffer_t buffer = {
            .buffer = (uint8_t *)buffer_tx,
            .capacity = sizeof(buffer_tx),
            .size = 0,
            .position = 0,
        };
        cmp_ctx_t cmp;
        cmp_init(&cmp, &buffer, NULL, NULL, buffer_writer);

        json_to_msgpack(&cmp, json);

        sendto(sock, buffer.buffer, buffer.size, 0, (struct sockaddr *)&send_addr,
               sizeof(send_addr));

        cJSON_Delete(json);
    }

    zmq_close(subscriber);
    close(sock);

    return NULL;
}

int main() {
    pthread_t receiver;
    pthread_t transmitter;

    pthread_create(&receiver, NULL, udp_receiver, NULL);
    pthread_create(&transmitter, NULL, udp_transmitter, NULL);

    pthread_join(receiver, NULL);
    pthread_join(transmitter, NULL);
}
