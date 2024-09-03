# {py:mod}`Kompass.topic`

```{py:module} Kompass.topic
```

```{autodoc2-docstring} Kompass.topic
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`Topic <Kompass.topic.Topic>`
  - ```{autodoc2-docstring} Kompass.topic.Topic
    :summary:
    ```
* - {py:obj}`AllowedTopic <Kompass.topic.AllowedTopic>`
  - ```{autodoc2-docstring} Kompass.topic.AllowedTopic
    :summary:
    ```
````

### Functions

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`create_topics_config <Kompass.topic.create_topics_config>`
  - ```{autodoc2-docstring} Kompass.topic.create_topics_config
    :summary:
    ```
* - {py:obj}`update_topics_config <Kompass.topic.update_topics_config>`
  - ```{autodoc2-docstring} Kompass.topic.update_topics_config
    :summary:
    ```
````

### API

````{py:class} Topic
:canonical: Kompass.topic.Topic

Bases: {py:obj}`auto_ros.topic.Topic`

```{autodoc2-docstring} Kompass.topic.Topic
```

````

````{py:class} AllowedTopic
:canonical: Kompass.topic.AllowedTopic

Bases: {py:obj}`auto_ros.topic.AllowedTopic`

```{autodoc2-docstring} Kompass.topic.AllowedTopic
```

````

````{py:function} create_topics_config(name: str, **kwargs) -> type[Kompass.config.BaseAttrs]
:canonical: Kompass.topic.create_topics_config

```{autodoc2-docstring} Kompass.topic.create_topics_config
```
````

````{py:function} update_topics_config(old_config_obj: Kompass.config.BaseAttrs, **kwargs)
:canonical: Kompass.topic.update_topics_config

```{autodoc2-docstring} Kompass.topic.update_topics_config
```
````
