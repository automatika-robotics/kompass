# Robust & Event-Driven Navigation

Kompass allows your robot to think in **reflexes** and **self-heal** when things go wrong.

These tutorials focus on the **Adaptive and Dynamic Orchestration** in Kompass. You will learn how to make your robot robust to failures and reactive to changing environments without hardcoding complex state machines inside your components.

:::{admonition} Recommended
:class: note
Complete one of the [basic navigation patterns tutorials](./index.md) before starting this part.
:::

## Tutorials

::::{grid} 1 1 2 2
:gutter: 3

:::{grid-item-card} {material-regular}`menu_book;1.5em;sd-text-primary` Understanding Fallbacks/Events/Actions
:link: events_actions
:link-type: doc

**Reflexes, not callbacks.**
Complete guide on defining custom Fallbacks/Events/Actions for your recipe
:::

:::{grid-item-card} {material-regular}`healing;1.5em;sd-text-primary` Self-Healing
:link: fallbacks_simple
:link-type: doc

**Robustness with Fallbacks**
"If the planner fails, don't crash—restart it."
Learn how to configure individual components to automatically recover from internal hardware disconnects and algorithm failures.
:::

:::{grid-item-card} {material-regular}`monitor_heart;1.5em;sd-text-primary` System Reflexes
:link: events_cross_healing
:link-type: doc

**Cross-Component Healing**
"If the Controller gets stuck, ask the Driver to unblock."
Learn to create system-level reflexes where components monitor each other's status to fix deadlock situations.
:::

:::{grid-item-card} {material-regular}`visibility;1.5em;sd-text-primary` External Reflexes
:link: events_external_reflexes
:link-type: doc

**Vision Reflexes**
"If a person is seen, follow them."
Create behavioral reflexes that react to external perception events, switching the robot's mode from patrolling to tracking.
:::

:::{grid-item-card} {material-regular}`hub;1.5em;sd-text-primary` Logic Gates
:link: events_composed
:link-type: doc

**Composed Conditions**
"If path is blocked **AND** battery is low..."
Use Python bitwise operators (`&`, `|`, `~`) to create smart composed events that fuse data from multiple topics.
:::

:::{grid-item-card} {material-regular}`auto_mode;1.5em;sd-text-primary` Dynamic Data
:link: events_dynamic
:link-type: doc

**Context-Aware Actions**
"Don't just detect—adapt."
Extract data from the triggering event and pass it dynamically to actions to reconfigure the robot on the fly.
:::
::::


```{toctree}
:maxdepth: 1
:caption: Event-Driven Navigation
:hidden:

events_actions
fallbacks_simple
events_cross_healing
events_external_reflexes
events_composed
events_dynamic
```
