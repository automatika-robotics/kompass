# {py:mod}`kompass.topic`

```{py:module} kompass.topic
```

```{autodoc2-docstring} kompass.topic
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`Topic <kompass.topic.Topic>`
  - ```{autodoc2-docstring} kompass.topic.Topic
    :summary:
    ```
* - {py:obj}`AllowedTopic <kompass.topic.AllowedTopic>`
  - ```{autodoc2-docstring} kompass.topic.AllowedTopic
    :summary:
    ```
````

### Functions

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`create_topics_config <kompass.topic.create_topics_config>`
  - ```{autodoc2-docstring} kompass.topic.create_topics_config
    :summary:
    ```
* - {py:obj}`update_topics_config <kompass.topic.update_topics_config>`
  - ```{autodoc2-docstring} kompass.topic.update_topics_config
    :summary:
    ```
````

### API

````{py:class} Topic
:canonical: kompass.topic.Topic

Bases: {py:obj}`auto_ros.topic.Topic`

```{autodoc2-docstring} kompass.topic.Topic
```

````

````{py:class} AllowedTopic
:canonical: kompass.topic.AllowedTopic

Bases: {py:obj}`auto_ros.topic.AllowedTopic`

```{autodoc2-docstring} kompass.topic.AllowedTopic
```

````

````{py:function} create_topics_config(name: str, **kwargs) -> type[kompass.config.BaseAttrs]
:canonical: kompass.topic.create_topics_config

```{autodoc2-docstring} kompass.topic.create_topics_config
```
````

````{py:function} update_topics_config(old_config_obj: kompass.config.BaseAttrs, **kwargs)
:canonical: kompass.topic.update_topics_config

```{autodoc2-docstring} kompass.topic.update_topics_config
```
````
