"""
Supported Events in Kompass

```{note}
Check [ROS Sugar](https://github.com/automatika-robotics/ros_sugar) for related API documentation
```


Events are created to alert a robot software stack to any dynamic change at runtime. An Event is defined by a change in a ROS2 message value on a specific topic.

Events are used by matching them to 'Actions'; an Action is meant to be executed at runtime once the Event is triggered.

Available Events:

```{list-table}
:widths: 20 80
:header-rows: 1
* - Event Class
  - Trigger

* - **OnEqual**
  - When a given topic attribute value is equal to a given trigger value

* - **OnDifferent**
  - When a given topic attribute value is different from a given trigger value

* - **OnChange**
  - When a given topic attribute changes in value from any initial value to any new value

* - **OnChangeEqual**
  - When a given topic attribute changes in value from any initial value to *given trigger goal* value

* - **OnGreater**
  - When a given topic attribute value is greater than a given trigger value

* - **OnLess**
  - When a given topic attribute value is less than a given trigger value
```

"""

from ros_sugar.core import Event, InternalEvent, OnInternalEvent
from ros_sugar.events import (
    OnChange,
    OnAny,
    OnChangeEqual,
    OnDifferent,
    OnEqual,
    OnGreater,
    OnLess,
)

__all__ = [
    "Event",
    "InternalEvent",
    "OnChange",
    "OnAny",
    "OnChangeEqual",
    "OnDifferent",
    "OnEqual",
    "OnGreater",
    "OnInternalEvent",
    "OnLess",
]
