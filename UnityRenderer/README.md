# Montage4D Summary
The commoditization of virtual and augmented reality devices and the availability of inexpensive consumer depth cameras have catalyzed a resurgence of interest in spatiotemporal performance capture.
Recent systems like Fusion4D and Holoportation address several crucial problems in the real-time fusion of multiple depth maps into volumetric and deformable representations.
Nonetheless, stitching multiview video textures on dynamic meshes remains challenging due to imprecise geometries, occlusion seams, and critical time constraints.
In this paper, we present a practical solution towards real-time seamless texture montage for dynamic multiview reconstruction.
We build on the ideas of dilated depth discontinuities and majority voting from the Holoportation project to reduce ghosting effects when blending textures.
In contrast to their approach, we use view-dependent rendering techniques to determine the appropriate textures for each vertex, in order to avoid the fuzziness caused by normal-based blending. 
By making use of discrete-differential-geometry-guided geodesics and temporal texture fields, our algorithm mitigates spatial occlusion seams while maintaining temporal consistency.
Experiments demonstrate significant enhancement in rendering quality, especially for detailed regions such as faces. 
We envision that Montage4D may be useful in a wide range of applications such as immersive telepresence for business, training, and live entertainment.

# Keys
## Main keys
- Press 1 to enable the original renderer, which runs at over 200 fps on the TitanX GPU.
- Press 2 to enable the Montage4D renderer, which provides sharper results at over 100 fps on the TitanX GPU.
- Press 3 to enable the Cartoon filter
- Press 4 to enable the view for normal vectors
- Press 5 to enable the view for using phong lighting
- Press 6 to enable the view with holographic effects
- Press 7 to enable the view with wireframes
- Press F12 to trigger the automatic calibration (reload the calibration files, and rebuild the compute shader)
## Debug mode (uncomment ENABLE_MONTAGE4D_DEBUG in RenderMontage4D.cginc)
- use the numeric keypads [0, RigConfiguration.NumPods) to observe the back-projection from a specific view, use the numeric keypad 8 to enable or disable fusion, use 9 to toggle the discontinuity test
- F1 - Show the color labels for the texture fields
- F2 - Show the Geodesic Fields by the compute shaders
- F3 - debugNormalWeightedField
- F4 - debugSeams
- F5 - showTexWeights
- F6 - debugNormalWeightedBlendingColor
- F7 - debugTextureWeights
- F9 - take a screen shot to the debug folder
- F10 - run the Spline-curved camera path for capturing
- Enter - temporal debug
- use ctrl and numeric keys for promoting a view globally, use alt and numeric keys for demoting a view globally

# Features
* Screenshots
    * Press C to create back-projection images from all cameras to debug whether there is defocus from the cameras, under the \\SolutionDir\Debug folder
    * Press F9 to capture a screenshot of the current renderer under the \\SolutionDir\Debug folder
* Camera Spline Path
    * Users can define custom fly path of the camera using spline nodes
    * In Main Camera Offset, drag your Spline root onto the M4D Spline Controller
* Sliders for tuning parameters
    * Spatial Diffusion Iterations
    * Normal Threshold for Seams
    * Normal Weighted Blending Opacity
    * Temporal Transition Speed (attention: the larger, the slower; it's the interval for the transition)
* Postprocessing (in MainCamera, enable the corresponding scripts to enable the postprocessing shaders) 
    * Minecraft
    * Cartoon
    * Minecraft
    * Sketch
