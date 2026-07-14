"""ws_protocol.py — message format for the frontend<->backend WebSocket.

Two frame types share one WS connection:

1. JSON control frames — text messages. Used for requests (frontend->backend)
   and responses/events (backend->frontend). Shape:
       {"type": <msg_type>, "id"?: <req_id>, ...payload}
   `id` is set on requests and echoed on responses so the frontend can match
   async replies. Events (server-initiated) omit `id`.

2. Binary scene frames — binary messages. Used for big arrays (point
   positions/colors) that would explode if JSON-encoded. A binary frame is
   preceded by a JSON text frame describing it:
       text : {"type":"scene_binary","frame_id":42,"ops":[...meta...],
               "layouts":[{"obj_id":"...","kind":"points",
                            "n_points":N,"has_colors":true}]}
       binary: concatenated float32 arrays in the order given by `layouts`
   See scene_bridge.serialize_scene for the exact byte layout.

Frontend->backend message types (requests):
    hello           -> {client, version}            ack
    list_plugins    -> {}                           catalog (list)
    enable_plugin   -> {name}                       {ok}
    disable_plugin  -> {name}                       {ok}
    set_property    -> {name,key,value}             {ok}
    get_state       -> {}                           {enabled, properties}

Backend->frontend message types (events, no id):
    ready           -> {}            backend initialized
    catalog         -> [plugin desc] plugin catalog
    state           -> {...}         enabled plugins + properties (after change)
    scene           -> {...}         JSON-only scene update (small ops)
    scene_binary    -> {...}         header for an upcoming binary frame
    log             -> {level,msg}   backend log line for the console panel
    robot_added     -> {...}         a robot connected (Phase 3)
    robot_status    -> {...}         connection/health update (Phase 3)
"""
from __future__ import annotations
import json
import itertools

_id_counter = itertools.count(1)


def next_id() -> int:
    return next(_id_counter)


def make_msg(msg_type: str, **payload) -> str:
    """Build a JSON text frame. For events (no request id)."""
    payload["type"] = msg_type
    return json.dumps(payload)


def make_response(req_id: int, **payload) -> str:
    payload["type"] = "response"
    payload["id"] = req_id
    return json.dumps(payload)


def make_error(req_id: int, message: str) -> str:
    return json.dumps({"type": "error", "id": req_id, "message": message})


def parse(text: str) -> dict:
    """Parse an incoming JSON text frame. Raises ValueError on bad JSON."""
    return json.loads(text)
