# Omniverse Guide

## Nucleus
- This is for navigating files and contains connectors.
- You can set up the collaboration by adding users, who inherit group permissions.
- You can manage permissions from here.
- You can add bookmarks to folders to faster access the data where you’re working.
- Nucleus server also has an integrated AI search which you can find in search and opening advanced options.
- You can search with images.
- You can drag checkpoints into the scene. The checkpoints could be saved by clicking `files → save with options → checkpoints` on the right-hand side.

## USD Composer
An Application that Navigates, Builds, Modifies, and Renders photo-accurate materials rendering in both.

### Tools:
- Animation Workflows
- Physics Simulations 
- Visual Scripting

### World Building
- We can turn assets like lightning into their own entities. Then we can import them into the scene as payloads by saving them from the layers tab and then editing them inside the scene without incurring any changes to the file. We can also turn off the payloads.
- You can select a layer to be a default prim where everything dragged or created will be placed by right-clicking and finding.
- Creating Layers allows team members to edit USD in their stages, and you can easily turn on or off their layer by clicking the eye icon.

### Materials
- **Create a new material**:
  1. Navigate to the top menu and click create.
  2. Look inside the material section.
  3. Click OmniPBR (Physically Based Rendering - default material)
  4. Change color if needed
  5. Drag and Drop into the mesh from the layer into the stage.
- **To edit an already existing material**:
  1. Navigate to the property’s panel and open it.

### Physics
- If you want the assets to sit on top of other assets we need to work with the zero gravity tool.
- Activate the zero gravity tool located on the left in the toolbar by clicking it.
- We need to add a few markers that will determine whether they are immovable or interact according to the physics during the simulation.
  - If we have a bowl with fruit in it: we select a bowl and click on the red box which sets the static marker.
  - Then we select the three fruits, making sure that we select the parent Xform that contains several meshes for each fruit so they move together respectively.
  - Then, we add a dynamic marker using the green cube, which will indicate that they will only stop once they interact with something (the bowl).

### Extensions
This is the Omni kit.
- Windows → Extensions 

## Content Browser Overview
- Window → Content Panel
- Easily browse in your computer file system as long as the local host.
- You can also import and convert to USD.
- Checkpoints are also available here to look at history or revert to the original design.
- Control and click the search path.
- There is also a search.

## Useful Keyboard Shortcuts for Navigation
- Uses kit view extension
  - Look Around: Right Click
  - Walk Forward: Right Click + W
  - Walk Backward: Right Click + S
  - Move Right: Right Click + D
  - Move Left: Right Click + A
  - Look up: Right Click + E
  - Drift Down: Right Click + Q
  - Change the Speed of Movement: Right Click + Wheel (rolly molly)
  - Fluid Zoom: Right Click + ALT and drag
  - Focus on an Object: select and hold F

## Cameras
To create a camera go to `Create → Camera`, or right click on the scene and choose `Create → Camera`.

## Types of Lights
- You have control over shadow, exposure, color.
  - Create → Light → [...] light
  - **Distant Lights** → Lights like from the sun, characterized with long and sharp shadows
  - **Sphere Light** → Like a light from a light bulb, no harsh shadows
  - Cylinder Lights
  - Dome Lights (preferred)

## Why USD?
### Types of Files
- **.USD files**: these are *.usdc, or crate files. These are binary files that are compressed and fast to access.
- **.USDZ Files**: Support for storing a collection of files in a zip archive. For example, a mesh and its textures.
- **.USDA Files**: A text version of a usd file that looks a little like a nested python dictionary.
- **Others**: File formats are provided by plugins. For example, *.drc files for Draco compression.

### Layer Composition is what is unique about USD files by combining different files.
- **Sublayers** → combine like Photoshop layers.
  - For non-destructively editing layers 
- **References** → Add a new item to your scene.
- **Payloads** → for switching between items in a collection.
- **Inherits** → like C++ classes
- **Specialized** → Inherits with some special properties.

## 5 Things to Know About USD
1. Made by Pixar
2. Open Source and Free (USD View, the software, is also free)
3. Fun Fact: Apple also uses it for AR
4. Omniverse extends USD to allow live collaboration across software.
5. USD enables complex asset structures that are shared between tools.

## Layers
Every layer in a scene is a new or a reference to a USD file.

“Layers form the building blocks upon which the scalable complex systems can be contained, managed, and stored.”

- Bottom to top, we can mute and unmute the layers to understand them.
- **The Layer Panel**
  - Manages USD References
  - Organizer in a nested list
  - Controls layer strength (top to bottom)
  - Allows for Opinions for the final scene representation

Best Practice: Lower ordered layers are best kept simple (no positions, no materials, etc).

### An Example of Layer Hierarchy
- Layer of the Background
- Layer of chess board and all the pieces together
- Layer where “deltas” all the changes to the scene are stored which can be deleted, and payloads that denote assets introduced in the layer (blue arrow).
- Layer of the Materials

### For Collaboration: We can add opinions and mute or unmute them to see and compare.

## Omniverse Live
- You can make live changes on USD files as long as they’re on the nucleus. This is useful for:
  - Group collaborations
  - Individual working with several software.
- When you start a live session a separate layer appears in the layer panel that will be independent from the root layer allowing for non-destructive editing. Once you’re done you can merge the edits with root or a new layer.

## Extensions
- You can easily browse extensions, by using filters, search, by importing them.

## Audio
- You can import audio on a speaker. Then you need to add frames because music plays with time.

## Warp
- You can import audio on a speaker. Then you need to add frames because music plays with time.

## Action Graphs
- Action Graphs are a visual scripting tool used to define how objects and properties within your scene should behave or react to specific events. Imagine them like flowcharts or recipes that tell your scene elements what to do in different situations.
- They rely on events as triggers. These events can be anything from user input (like a key press) to changes in the scene itself (like a collision between objects).
- Action Graphs use a node-based interface. You connect nodes together to create a chain of actions. Each node represents a specific operation, such as reading a user input, modifying a property, or triggering another event.
- Action Graphs allow you to create complex behaviors for your scene elements without needing to write traditional code. This makes them very accessible even for non-coders.

## Animation Graph
- From Isaac assets import a human into the scene
- Add `Window → Animation → Animation Script`
- Create new animation and choose the root as the skeleton
- Add the animations available in the human prim like walk and stand.
- Add blend and connect standing to pose 0 and walking to pose 1.
- Connect blend pose to the final animation pose.
- Add the variable called walk and connect it to blend.
- Add the animation graph into the skelRoot by `Right Clicking on it → Add → Animation → Animation Script` choose the one we just created. It will show up in the property panel.
- It already might be in the property panel and the target of the animation script might be different. In this case, change the target straight from the property panel.
- Add action graph from `Window → Visual Scripting → Action Graph`
- Create a new action graph.
- Add `On Keyboard Input`.
- Add two `Write Animation Graph Variables`.
- For both Write Animation Graph Variable Nodes set the variable name to walking.
- Connect the On Keyboard Input Node with Write Animation Graph Variable Node using Exec in port.
- Add two Constant Floats and connect them to Write Animation Graph Variable Nodes through value nodes.
- Set the value of one Constant Float node to 1, while the other to 0.
- Add Get Graph Target and connect them to SkelRoot Path.
- Add the Action Graph to skelRoot by `Right Clicking on it → Add → Visual Scripting` (Note: In the tutorial there is an outdated name for this Omnigraph). → choose the one we created.
- Now the property panel should have visual scripting and animation graphs.
- Press Play and Press key `a` to walk, release to stop.

## Animation Timeline and Curve Editor Overview
- You can match the pose of many characters using Animation Retargeting and match.
- You can do a preview before saving the file.
- We need to supply animations and bind it to the root using `Add → Animation → Skeletal Binding → Add Target → Choose the desired root`.
- Add binding between the character and the animation.

## Paint Tool Extension Overview
Painting live is possible with different objects

