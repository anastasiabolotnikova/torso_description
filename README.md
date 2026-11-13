torso_description
==================

Geometry representing a generic human torso, in two versions:

| URDF | Links | Load in mc_rtc |
| --- | --- | --- |
| `urdf/torso.urdf` | torso and head, one fixed body | `[object, <share path>, torso]` |
| `urdf/torso_arms.urdf` | the same torso plus two articulated arms with open palms (14 joints) | `[object, <share path>, torso_arms]` |

Both have the same surfaces (`rsdf/torso`, `rsdf/torso_arms`), including `Front` for mounting a
robogami arm.

### torso_arms

Each arm has the shoulder (`*Arm_0..2`), elbow (`*Elbow`), forearm twist (`*Forearm`) and wrist
(`*Wrist_0..1`) joints of `human_description`'s `urdf/human.urdf`, with its joint limits, masses
and centers of mass. At zero joint angles the arms point straight out to the sides; `LArm_0 = -1.4`
and `RArm_0 = 1.4` let them hang down.

The meshes come from `human_description`, like the torso mesh: each arm link mesh is decimated on
its own (to 50-120 faces), and each hand is a single open-palm mesh (`meshes/LPalm.stl`,
`meshes/RPalm.stl`): `human_description`'s open hand cut at the wrist, with the thumb turned 30
degrees closer to the fingers, and replaced by its convex hull (so the visual mesh and the collision
hull are the same), in place of separate wrist and finger meshes. To regenerate them and the URDF:

```
python3 scripts/generate_torso_arms.py [path/to/human_description]
```

### Previewing RSDF surfaces in RViz

```
python3 scripts/visualize_rsdf_surfaces.py rsdf/torso/torso.rsdf
```

Then in RViz, add a `MarkerArray` display subscribed to `/rsdf_surfaces` (alongside the mesh via
`robot_state_publisher`, Fixed Frame = `torso`)