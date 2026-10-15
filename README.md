torso_description
==================

Geometry representing a generic human torso.

### Previewing RSDF surfaces in RViz

```
python3 scripts/visualize_rsdf_surfaces.py rsdf/torso/torso.rsdf
```

Then in RViz, add a `MarkerArray` display subscribed to `/rsdf_surfaces` (alongside the mesh via
`robot_state_publisher`, Fixed Frame = `torso`)