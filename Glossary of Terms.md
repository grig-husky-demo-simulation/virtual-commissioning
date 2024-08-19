# Glossary of Terms

## USD Terms

**Universal Scene Description (USD)** → Originally developed by Pixar Animation Studios, USD, aka OpenUSD, is more than a file format - it's an open-source 3D scene description used for 3D content creation and interchange between tools. Its versatility has made it an industry-standard in visual effects, architecture, robotics, manufacturing, and more.

**Prim** → A Prim is the primary container object in USD: prims can contain (and order) other prims, creating a “namespace hierarchy” on a Stage, and prims can also contain (and order) properties that hold meaningful data. This is the basic building block of a scene in Omniverse. It can represent various things like a mesh, a light, or even a group of other prims.

**LayerStack** → This refers to the layered structure of a scene in Omniverse. Each layer can contain different elements like geometry, materials, etc.

**Payload** → A Payload is a specialized type of reference used to incorporate assets or layers into a scene. Unlike references, which automatically load all linked assets during scene opening, payloads allow for selective loading. You can choose to defer loading the referenced asset until it's specifically needed, improving performance for large or complex scenes. If a prim has both a reference and a payload pointing to the same asset, the reference takes priority and loads automatically. Payloads offer more flexibility for managing asset dependencies.

**Component-Model Assets** → These are assets that are built from smaller, reusable pieces. Imagine a car model being assembled from individual components like wheels, chassis, etc. Payloads are particularly useful for managing these types of assets by allowing you to load individual components as needed.

**Metadatum** → Extra information attached to a prim, like a label or a note.

**Attribute** → A specific property of a prim, like its color, size, or material (think of it as a characteristic).

**Relationship** → How prims connect to each other, like parenting one block to another (imagine how blocks can be stacked or linked).

**Opinion** → In USD, opinions are like individual suggestions for how a prim's properties (attributes) should be set. Imagine building a scene with blocks. Each block can have suggestions for its color, size, etc. These suggestions are the opinions. There can be many opinions for the same property, coming from different places in your scene. A special order (LIVRPS) acts like a hierarchy, deciding which suggestion "wins'' and determines the final look of your block (prim). Think of LIVRPS as a priority list for suggestions, with local changes on the block itself having the highest priority.
- **Local**: Highest priority - Changes made directly on the prim itself.
- **Inherits**: Properties "inherited" from parent prims.
- **VariantSets**: Different property variations based on selected sets.
- **References**: Properties defined in referenced external assets.
- **Payload**: Properties from loaded payloads (special references). (Only applies if loaded)
- **Specializes**: Highly specific overrides for the prim (full LIVRPS check).
- **No Opinion Found**: If none of the above provide a value, the default applies.

## Omniverse Terms

**Zero Gravity Markers** → This refers to a specific extension for Omniverse that allows you to define how objects in your scene behave during simulations. There are two main types of markers in this context:
- **Static Markers** → These markers designate objects that are immovable and won't be affected by physics simulations. Think of them like anchoring objects to the ground.
- **Dynamic Markers** → These markers identify objects that can move and interact with other objects during simulations. It's like setting objects free to behave realistically based on physics.

**Xform** → Short for transformation, is a fundamental concept in Omniverse. It's a property attached to every prim (building block) in a scene, defining its position, orientation, and scale. This essentially determines where a prim is located, how it's rotated, and its size within the 3D world. Understanding Xform is crucial for creating the layout and animation of your scenes in Omniverse.

**Rendering** → Refers to the process of generating visual representations from 3D data. Specifically, it involves creating images or animations based on 3D scenes, materials, lighting, and camera viewpoints.

## General Terms

**Draco** → Draco is a library for compressing and decompressing 3D geometric meshes and point clouds. It is intended to improve the storage and transmission of 3D graphics. Draco was designed and built for compression efficiency and speed.

**HDRI** → HDRI stands for High Dynamic Range Image. It's a type of image used in computer graphics, especially 3D rendering, that captures a wider range of light and color information than a standard photograph. 
Imagine a scene with a bright sky and a dark foreground. A regular photo might struggle to capture both extremes well, with details lost in the highlights or shadows. An HDRI image, on the other hand, can capture this wider range of light values, preserving details in both bright and dark areas.
When used in 3D rendering software, HDRI images provide realistic lighting for your scene. The software can use the lighting information from the HDRI to illuminate your 3D models, creating natural-looking shadows, reflections, and highlights. 
Fun Fact: Used in CGI to put an imaginary city's reflection on cars for example.
HDRI images often capture a full 360-degree view of a scene, including the sky, ground, and surrounding objects. This allows you to create a realistic environment for your 3D models, making them appear as if they're part of a real-world location.



