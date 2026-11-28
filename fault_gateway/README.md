# fault_gateway

Web API for the fault manager: REST, a live event stream (server-sent events)
and a status page. Python, standard library only.

Endpoints, examples and the security model:
[docs/fault_management.md](../docs/fault_management.md#fault_gateway-the-web-api).

```bash
ros2 launch fault_gateway fault_gateway.launch.py
# then open http://localhost:8080/
```

| Parameter | Default | Meaning |
|---|---|---|
| `host` | `127.0.0.1` | Address to listen on. `0.0.0.0` serves the network |
| `port` | `8080` | |
| `api_token` | empty | When set, acknowledging needs `Authorization: Bearer <token>` |
| `service_timeout_s` | `2.0` | How long to wait for the fault manager before answering 503 |

There is no TLS. On an untrusted network, put a reverse proxy in front.

## Tests

`http_api.py` and `convert.py` do not import ROS, so their tests run anywhere:

```bash
cd fault_gateway && python3 -m pytest test/test_http_api.py test/test_convert.py
```
