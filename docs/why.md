# Why Kompass?

Robotic navigation isn't about perfecting a single component; it is about architecting a system that survives contact with the real world.

While metric navigation has matured, deploying robots extensively in dynamic environments remains an unsolved challenge. As highlighted by the **ICRA BARN Challenges**, static pipelines fail when faced with the unpredictability of the physical world:

> _"A single stand-alone approach that is able to address all variety of obstacle configurations all together is still out of our reach."_
> — **Lessons from The 3rd BARN Challenge (ICRA 2024)**

**Kompass was built to fill this gap.** Unlike existing solutions that rely on rigid behavior trees, Kompass is an event-driven, GPU-native stack designed for maximum adaptability and hardware efficiency.

## 1. True Adaptive Navigation (Event-Driven Core)

Kompass adheres to open event-driven software standards, allowing the navigation stack to **reconfigure itself on the fly**.

- **Dynamic Response:** The stack adapts not just to internal robot states (like Battery Low), but to external world events (e.g., "Crowd Detected" or "Entering Warehouse").
- **Context-Aware Control:** You can configure the system to use a _Pure Pursuit_ controller on open roads, switch to _MPC_ indoors, and fallback to a precise docking controller—all triggered by environmental context.
- **Simplified Logic:** Unlike complex Behavior Trees that can become unmanageable, Kompass allows you to define clean, event-based transitions and fallback behaviors for every component.

## 2. High-Performance & Vendor-Agnostic GPU Acceleration

Kompass is engineered for speed, utilizing C++ for core algorithms and multi-threading for CPU tasks. However, its standout feature is its approach to hardware acceleration.

- **SYCL-Based Architecture:** Kompass is the **first navigation framework to support cross-GPU acceleration**.
- **Hardware Freedom:** Unlike CUDA-locked solutions, Kompass runs natively on **Nvidia, AMD, Intel and other integrated** GPUs/APUs without device-specific hacks.
- **Parallel Power:** Compute-heavy tasks like path planning, control, and map updates are offloaded to the GPU, freeing up your CPU for high-level logic.

## 3. ML Models as First-Class Citizens

Because Kompass is event-driven, it can directly utilize the outputs of Machine Learning models to drive behavior.

- **Neuro-Symbolic Control:** Use an object detection model to trigger a controller switch (e.g., switching to _Human-Aware TEB_ when people are detected).
- **VLM Integration:** Use Vision Language Models to answer abstract queries (e.g., "Am I indoors or outdoors?") and fundamentally alter the robot's planning strategy based on the answer.
- **Deep Integration:** See our tutorial on using [Vision Tracking with EmbodiedAgents](https://www.google.com/search?q=./tutorials/vision_tracking.md) to see this in action.

## 4. Pythonic Simplicity, ROS Compatible

We bridge the gap between high-performance C++ and developer-friendly Python.

- **One-Script Configuration:** Using [Sugarcoat🍬](https://www.github.com/automatika-robotics/sugarcoat), you can configure a sophisticated, multi-fallback navigation system in a single, readable Python script.
- **Clean Architecture:** Core algorithms (Kompass Core) are decoupled from ROS wrappers. This ensures that upgrading ROS distributions (or switching middleware) doesn't break your underlying navigation logic.
- **Extensible:** The modular design allows the community to easily plug in new planners or controllers in Python for prototyping or C++ for production.
