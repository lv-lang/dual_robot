RViz rendering overlay
======================

This directory is installed as a small, RViz-only ament prefix before the
system `rviz_rendering` prefix.

It exists to avoid a Mesa/Ogre shader link failure in the Humble Map display:

`active samplers with a different type refer to the same texture image unit`

Most files are copied unchanged from `/opt/ros/humble/share/rviz_rendering` so
RViz can resolve the full `ogre_media` resource directory from the overlay. The
intentional local changes are limited to:

- `share/rviz_rendering/ogre_media/materials/glsl120/indexed_8bit_image.frag`
- `share/rviz_rendering/ogre_media/materials/scripts/indexed_8bit_image.material`

The shaders keep RViz's real `sampler2D` map texture and `sampler1D` palette
texture, but move this material to GLSL 4.20 compatibility and explicitly bind
the samplers to texture units 0 and 1. This avoids the Mesa link-time sampler
conflict without breaking the palette lookup. The workaround stays local to
`robot_nav` RViz launches and does not modify `/opt/ros`.
