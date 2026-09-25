---
title: "Extending the Browser UI"
---

# Extending the Browser UI

`launcher.enable_ui(...)` serves a browser front-end and a JSON/WebSocket API for the topics, services and actions a recipe exposes. Sugarcoat knows how to render its own types; a package built on it registers elements for **its own** types through `UI_EXTENSIONS`. Kompass does that for the `MissionStatus` message and for the `MultiGoalPlanPath` action, whose card it owns outright.

## The extension point

```python
# kompass/kompass/ros.py
from ros_sugar import UI_EXTENSIONS


def augment_ui():
    """Imported lazily: the elements need the browser UI dependencies, which
    are only required when the UI is served with a browser front-end"""
    from .ui_elements import INPUT_ELEMENTS, OUTPUT_ELEMENTS, TASK_ELEMENTS

    return INPUT_ELEMENTS, OUTPUT_ELEMENTS, TASK_ELEMENTS


UI_EXTENSIONS["kompass"] = augment_ui
```

The launcher calls every registered extension when the UI is enabled with `serve_browser=True`, and skips them entirely in API-only mode -- which is why the import is inside the function: FastHTML and MonsterUI are optional dependencies.

Returning two dicts is enough; the third, the task cards, is optional.

| Dict | Keyed by | Value |
|---|---|---|
| `INPUT_ELEMENTS` | a `SupportedType` | callable building the widget that publishes to a topic |
| `OUTPUT_ELEMENTS` | a `SupportedType` | callable rendering an incoming message in the logging card |
| `TASK_ELEMENTS` | a ROS action type | a `Task` subclass owning that action's card |

:::{admonition} The UI runs in its own process
:class: important

The launcher does not hand over the callables: it serializes each key and value as a `module.qualname` string and the UI node imports them back. So every element has to be a **module-level name** in an importable module -- no closures, no locally built `partial`s -- and the type it is keyed by has to be a registered type (see [Data Types, Callbacks & Publishers](./custom_callbacks_publishers.md)), since the UI node resolves message types by name too.
:::

## Output elements

An output element takes the logging card, the message, and the name of the data source, and returns the card:

```python
def _log_mission_status_element(logging_card, output: ROSMissionStatus, data_src: str):
    signature = ...  # the fields that make one entry different from the last
    style = LOG_STYLES.get(data_src, DEFAULT_STYLE)
    label, badge = _MISSION_STATES.get(output.state, ("unknown", "unknown"))
    entry = Div(
        Strong(f"{style['prefix']} ", cls=style["cls"]),
        Span(label, cls=f"status-badge {badge}"),
        cls="whitespace-pre-wrap ml-2 p-2 flex flex-wrap items-center gap-2",
        id=_MISSION_STATUS_ID,
        data_signature=signature,
    )
    return logging_card(entry)


# Keyed by the Kompass SupportedType, while the element itself is handed the
# ROS message
OUTPUT_ELEMENTS = {MissionStatus: _log_mission_status_element}
```

Two conventions worth copying:

- **Style from `data_src`.** `LOG_STYLES` carries the prefix and colour of each source, with `DEFAULT_STYLE` as the fallback, so entries from different components stay distinguishable.
- **Say it once.** A status topic is often republished on a timer whether or not anything changed -- `mission_status` is, at the mission's cursor poll rate. Giving the entry a fixed `id` and a `data_signature` of the fields that matter lets the element find its own last entry in the card and skip an identical one.

Badge classes come from the Sugarcoat stylesheet: `status-badge` plus `running`, `accepted`, `active`, `canceled`, `completed`, `aborted`, `paused` or `inactive`.

## Task cards

Every action client the UI knows about gets a generic `Task` card: a form built from the goal fields, a status badge, and a log of feedback messages. Registering a class in `TASK_ELEMENTS` under an action type replaces that card with your own:

```python
class MissionTask(Task):
    ...

TASK_ELEMENTS = {ROSMultiGoalPlanPath: MissionTask}
```

The hooks a subclass overrides:

| Member | Purpose |
|---|---|
| `__init__(name, client_type, fields)` | Set up card state before calling `super().__init__` |
| `update(*, status, feedback, duration, timestep)` | Called for each goal status change and each feedback message |
| `cleanup()` | A new goal starts with a clean card |
| `card` | The whole card, rebuilt on every redraw |
| `_badge` | The status badge, by default the goal's status |
| `_feedback_card(title=...)` | The built-in feedback log, if you still want one |
| `is_active()`, `_status`, `_feedback`, `_duration`, `_timer` | State the base class keeps for you |
| `_dom_id` | Prefix for every DOM id the card uses, so two cards never collide |

`MissionTask` is the worked example. Its `update()` keeps the latest feedback as the card's own state and passes only *changes* to the feedback log, since mission feedback arrives several times a second and mostly says the same thing. Its `card` renders the journey as a checklist, the pause/resume/cancel controls, the log, and the form where the next mission is put together. Its `_badge` reports what the *mission* is doing rather than what the goal is doing: a paused mission's goal is still running, as far as the action server is concerned.

:::{admonition} Why the card belongs to the action
:class: note

A mission is carried out by a routine, and routines in Sugarcoat are not tied to action servers -- a routine card cannot assume there is a goal to send, and the generic action card cannot assume there is a routine to follow. Owning the `MultiGoalPlanPath` card in Kompass, where both are known to belong to the same component, keeps that knowledge out of the framework.
:::

## Talking to the UI's API from a card

A card renders HTML, so anything interactive is a `Script` in the card that calls the same JSON API the rest of the UI uses:

| Endpoint | Purpose |
|---|---|
| `GET /api/interfaces` | What this UI serves: topics, services and actions, with their types |
| `POST /api/actions/<name>` | Send a goal, as JSON |
| `POST /api/actions/<name>/cancel` | Cancel the goal in flight |
| `WS /api/actions/<name>/feedback` | Feedback stream |
| `POST /api/services/<name>` | Call a service |
| `POST /api/inputs/<topic>` | Publish to an input topic |
| `WS /api/outputs/<topic>` | Subscribe to an output topic |
| `GET /api/outputs/<topic>/latest` | The last message on an output topic |

Names depend on the recipe, so a card should **discover** rather than hard-code them: the mission card asks `/api/interfaces` for the service whose type is `ExecuteMethod` and for its waypoints topic, and falls back to doing nothing when the recipe did not expose them. Its own action name is the one thing it knows, substituted into the script when the card is built.

## Declaring what a card needs

Anything a card talks to has to be exposed to the UI by the recipe. Gather that on the component instead, so recipes stay short:

```python
@property
def ui_inputs(self) -> List[Any]:
    """The action a mission is sent to, the service the card pauses and
    resumes through, and the topic it collects waypoints from"""
    return [
        self.ui_main_action_input,
        ServiceClientConfig(name=f"{self.node_name}/execute_method",
                            srv_type=ExecuteMethod),
        self.ui_waypoints,
    ]
```

```python
launcher.enable_ui(
    inputs=[clicked_point_topic, *mission.ui_inputs],
    outputs=[map_topic, odom_topic, *mission.ui_outputs],
    serve_browser=True,
)
```

A topic can be declared both ways on purpose: the mission's waypoints topic is an input so the map can publish a click on it, and an output so the card can read the clicks back and collect them into a journey.

## Testing

UI elements are plain functions and classes rendering to FastHTML tags, so they test without a node or a browser -- register them the way the launcher and the UI node do, then assert on the rendered XML:

```python
_, outputs, _tasks = UI_EXTENSIONS["kompass"]()
elements.add_additional_ui_elements(
    input_elements=[],
    output_elements=[
        (f"{k.__module__}.{k.__qualname__}", f"{e.__module__}.{e.__qualname__}")
        for k, e in outputs.items()
    ],
)

card = elements.initial_logging_card()
card = elements.update_logging_card(card, status_msg, "MissionStatus", "robot")
assert "navigating" in to_xml(card)
```

Going through the serialized dotted paths is the point: it is the same step the launcher performs, so a test catches an element that the UI node would not be able to import. See `kompass/test/test_ui_elements.py`.

The one part this does not cover is the JavaScript inside a card, which only runs in a browser.
