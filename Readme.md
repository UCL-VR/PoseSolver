The PoseSolver library contains a set of tools to perform motion tracking and capture, through optimisation of a Pose from a set of observations, such as mocap markers.

Pose estimation is preformed by the PoseSolver C++ library. This library contains a number of classes, such as those to handle describing Transforms in 3D, and ways to describe kinematic constraints such as Denavit Hartenberg joints. The library uses these building blocks to express common tracking problems, such as hand tracking, as a factor-graph optimisation problem. The optimisation itself is performed by Google's Ceres solver. A number of tracking problems are included. Each is contained in its own class, which exports a C-style API for use in platforms such as Unity.

The library use/layout is as follows:


Utility Types

Pose3D DHParameters KinematicJoints 

Hand1 Hand2 FullBody1 -> C-API -> Unity -> Observations in/poses out

Ceres

Scene


The C++ solution also contains a set of Unit Tests and a Host program that can be used to utilities the library in various ways (all in C++).

In most cases however, the library is expected to be used with a 3D platform such as Unity. This is because it is easier to configure and perceive 3D problems in 3D, and tools like Unity have effective Editors for building scene graphs and adjusting things in 3D.

In addition to the PoseSolver library and Unity, the TrackerManager project can be used to connect to various data sources, record them and/or send them in a common format to Unity or Matlab. There are also example Matlab scripts, and assets such as meshes and trajectories.

The PoseSolver Library relies on C++ 20 and Visual Studio 2022. Earlier versions are not supported, though it is always possible to interact via the C-API. 


Adding a new Module

Problems exposed externally should be in the Modules section. Each entity in this section should consist of one class, within its own namespace, and a set of exported functions beginning with the namespace name. For example, the Hand1 solver lives in the namespace hand1 (hand1::Hand), and all its exported methods look like: hand1_myMethod().

Almost certainly, such modules will have counterparts in other platforms for getting data in and out via the C-Style API. In Unity, it is expected each module has a Component that imports all its C-Style APIs.


Unity Components should subclass from MonoBehaviour, rather than each other. The expectation is that only methods belonging to one namespace are used within one class at a time. This will involve sometimes re-exposing logic or duplicating methods that could be made more general by exposing, say, some of the DH methods to C.
However, the helper types in the C++ library are just helpers, and can be used in subtly different ways. For example, there may be a function setDHJointLimit that would appear to be best placed in a common set of methods. However, one solver may want to use the Residual to enforce this, and another a Manifold. Allowing the semantic of "limit" to be specific to a particular solver allows the implementation to change with equal specificity.

