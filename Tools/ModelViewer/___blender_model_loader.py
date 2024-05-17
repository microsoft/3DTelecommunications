import bpy
import os
import sys
from math import radians
import pdb; 

argv = sys.argv
argv = argv[argv.index("--") + 1:]  # get all args after "--"
version = "1.1.0"

print("\n\n** Using Model loader " + version + " **\n\n")

from bpy_extras.image_utils import load_image  # pylint: disable=import-error

def create_image_material( image, matName="Image", shading = 'SHADELESS'):
    material = bpy.data.materials.new(matName)
    material.use_nodes = True
    node_tree = material.node_tree
    output_node = clean_node_tree(node_tree)

    material.blend_method = 'BLEND'
    material.shadow_method = 'HASHED'

    texture_node = node_tree.nodes.new('ShaderNodeTexImage')
    im = load_image(image, check_existing=True, force_reload=True)
    texture_node.image = im
    texture_node.show_texture = True
    texture_node.extension = 'CLIP'

    # Create tree
    create_color_alpha_tree( node_tree, texture_node.outputs[0], texture_node.outputs[1], output_node.inputs[0], shading)

    return material, texture_node

def get_input_nodes(node, links):
    """Get nodes that are a inputs to the given node"""
    # Get all links going to node.
    input_links = {lnk for lnk in links if lnk.to_node == node}
    # Sort those links, get their input nodes (and avoid doubles!).
    sorted_nodes = []
    done_nodes = set()
    for socket in node.inputs:
        done_links = set()
        for link in input_links:
            nd = link.from_node
            if nd in done_nodes:
                # Node already treated!
                done_links.add(link)
            elif link.to_socket == socket:
                sorted_nodes.append(nd)
                done_links.add(link)
                done_nodes.add(nd)
        input_links -= done_links
    return sorted_nodes

def clean_node_tree(node_tree):
    """Clear all nodes in a shader node tree except the output.
    Returns the output node
    """
    nodes = node_tree.nodes
    for node in list(nodes):  # copy to avoid altering the loop's data source
        if not node.type == 'OUTPUT_MATERIAL':
            nodes.remove(node)

    return node_tree.nodes[0]


def get_shadeless_node(dest_node_tree):
    """Return a "shadless" cycles node, creating a node group if nonexistant"""
    try:
        node_tree = bpy.data.node_groups['Shadeless']

    except KeyError:
        # need to build node shadeless node group
        node_tree = bpy.data.node_groups.new('Shadeless', 'ShaderNodeTree')
        output_node = node_tree.nodes.new('NodeGroupOutput')
        input_node = node_tree.nodes.new('NodeGroupInput')

        node_tree.outputs.new('NodeSocketShader', 'Shader')
        node_tree.inputs.new('NodeSocketColor', 'Color')

        # This could be faster as a transparent shader, but then no ambient occlusion
        diffuse_shader = node_tree.nodes.new('ShaderNodeBsdfDiffuse')
        node_tree.links.new(diffuse_shader.inputs[0], input_node.outputs[0])

        emission_shader = node_tree.nodes.new('ShaderNodeEmission')
        node_tree.links.new(emission_shader.inputs[0], input_node.outputs[0])

        light_path = node_tree.nodes.new('ShaderNodeLightPath')
        is_glossy_ray = light_path.outputs['Is Glossy Ray']
        is_shadow_ray = light_path.outputs['Is Shadow Ray']
        ray_depth = light_path.outputs['Ray Depth']
        transmission_depth = light_path.outputs['Transmission Depth']

        unrefracted_depth = node_tree.nodes.new('ShaderNodeMath')
        unrefracted_depth.operation = 'SUBTRACT'
        unrefracted_depth.label = 'Bounce Count'
        node_tree.links.new(unrefracted_depth.inputs[0], ray_depth)
        node_tree.links.new(unrefracted_depth.inputs[1], transmission_depth)

        refracted = node_tree.nodes.new('ShaderNodeMath')
        refracted.operation = 'SUBTRACT'
        refracted.label = 'Camera or Refracted'
        refracted.inputs[0].default_value = 1.0
        node_tree.links.new(refracted.inputs[1], unrefracted_depth.outputs[0])

        reflection_limit = node_tree.nodes.new('ShaderNodeMath')
        reflection_limit.operation = 'SUBTRACT'
        reflection_limit.label = 'Limit Reflections'
        reflection_limit.inputs[0].default_value = 2.0
        node_tree.links.new(reflection_limit.inputs[1], ray_depth)

        camera_reflected = node_tree.nodes.new('ShaderNodeMath')
        camera_reflected.operation = 'MULTIPLY'
        camera_reflected.label = 'Camera Ray to Glossy'
        node_tree.links.new(camera_reflected.inputs[0], reflection_limit.outputs[0])
        node_tree.links.new(camera_reflected.inputs[1], is_glossy_ray)

        shadow_or_reflect = node_tree.nodes.new('ShaderNodeMath')
        shadow_or_reflect.operation = 'MAXIMUM'
        shadow_or_reflect.label = 'Shadow or Reflection?'
        node_tree.links.new(shadow_or_reflect.inputs[0], camera_reflected.outputs[0])
        node_tree.links.new(shadow_or_reflect.inputs[1], is_shadow_ray)

        shadow_or_reflect_or_refract = node_tree.nodes.new('ShaderNodeMath')
        shadow_or_reflect_or_refract.operation = 'MAXIMUM'
        shadow_or_reflect_or_refract.label = 'Shadow, Reflect or Refract?'
        node_tree.links.new(shadow_or_reflect_or_refract.inputs[0], shadow_or_reflect.outputs[0])
        node_tree.links.new(shadow_or_reflect_or_refract.inputs[1], refracted.outputs[0])

        mix_shader = node_tree.nodes.new('ShaderNodeMixShader')
        node_tree.links.new(mix_shader.inputs[0], shadow_or_reflect_or_refract.outputs[0])
        node_tree.links.new(mix_shader.inputs[1], diffuse_shader.outputs[0])
        node_tree.links.new(mix_shader.inputs[2], emission_shader.outputs[0])

        node_tree.links.new(output_node.inputs[0], mix_shader.outputs[0])

        auto_align_nodes(node_tree)

    group_node = dest_node_tree.nodes.new("ShaderNodeGroup")
    group_node.node_tree = node_tree

    return group_node

def auto_align_nodes(node_tree):
    """Given a shader node tree, arrange nodes neatly relative to the output node."""
    x_gap = 200
    y_gap = 180
    nodes = node_tree.nodes
    links = node_tree.links
    output_node = None
    for node in nodes:
        if node.type == 'OUTPUT_MATERIAL' or node.type == 'GROUP_OUTPUT':
            output_node = node
            break

    else:  # Just in case there is no output
        return

    def align(to_node):
        from_nodes = get_input_nodes(to_node, links)
        for i, node in enumerate(from_nodes):
            node.location.x = min(node.location.x, to_node.location.x - x_gap)
            node.location.y = to_node.location.y
            node.location.y -= i * y_gap
            node.location.y += (len(from_nodes) - 1) * y_gap / (len(from_nodes))
            align(node)

    align(output_node)

def create_color_alpha_tree(node_tree,color_input, alpha_input, shader_output, shading='SHADELESS'):
    # Create Shader
    if shading == 'PRINCIPLED':
        shaderNode = node_tree.nodes.new('ShaderNodeBsdfPrincipled')
    elif shading == 'SHADELESS':
        shaderNode = get_shadeless_node(node_tree)
    else:  # Emission Shading
        shaderNode = node_tree.nodes.new('ShaderNodeEmission')
    
    # Connect color
    node_tree.links.new(shaderNode.inputs[0], color_input)
    
    # Alpha
    alpha_node = node_tree.nodes.new('ShaderNodeMath')
    alpha_node.operation = 'MULTIPLY'
    alpha_node.inputs[1].default_value = 1
    alpha_node.label = "Opacity"
    alpha_node.name = "Opacity"
    if shading == 'PRINCIPLED':
        node_tree.links.new(alpha_node.inputs[0], alpha_input)
        node_tree.links.new(shaderNode.inputs[18], alpha_node.outputs[0])
    else:
        bsdf_transparent = node_tree.nodes.new('ShaderNodeBsdfTransparent')

        mix_shader = node_tree.nodes.new('ShaderNodeMixShader')
        node_tree.links.new(alpha_node.inputs[0], alpha_input)
        node_tree.links.new(mix_shader.inputs[0], alpha_node.outputs[0])
        node_tree.links.new(mix_shader.inputs[1], bsdf_transparent.outputs[0])
        node_tree.links.new(mix_shader.inputs[2], shaderNode.outputs[0])
        shaderNode = mix_shader
    
    # Connect to output
    node_tree.links.new(shader_output, shaderNode.outputs[0])
    
    # Align
    auto_align_nodes(node_tree)


if len(argv) <= 0:
    print("No model specified")
else:
    model_file = argv[0]
    context = bpy.context

    # Don't show the splash on load
    context.preferences.view.show_splash = False

    # models_path = "//"
    # models_path = os.path.dirname(os.path.realpath(__file__)) + "/"
    models_path=""

    render_path = "//"

    models = [model_file]

    #create a scene
    scene = bpy.data.scenes.new("Scene")
    # camera_data = bpy.data.cameras.new("Camera")

    # camera = bpy.data.objects.new("Camera", camera_data)
    # camera.location = (-2.0, 3.0, 3.0)
    # camera.rotation_euler = ([radians(a) for a in (422.0, 0.0, 149)])

    # scene.objects.link(camera)
    # scene.collection.objects.link(camera)

    # do the same for lights etc
    # scene.update()
    context.view_layer.update()

    for model_path in models:
        # scene.camera = camera
        path = os.path.join(models_path, model_path)
        # make a new scene with cam and lights linked
        # context.screen.scene = scene
        context.window.scene = scene
        
        # bpy.ops.scene.new(type='LINK_OBJECTS')
        bpy.ops.scene.new(type='LINK_COPY')
        context.scene.name = model_path
        # cams = [c for c in context.scene.objects if c.type == 'CAMERA']
        #import model
        bpy.ops.import_scene.obj(filepath=path, axis_forward='-Z', axis_up='Y', filter_glob="*.obj;*.mtl")


        found_mat = None
        found_tex = None
        found_output = None
        # correct the loading of the texture interpolation
        for mat in bpy.data.materials:
            if not mat.node_tree:
                continue
            for node in mat.node_tree.nodes:
                if node.type == 'TEX_IMAGE':
                    node.interpolation = 'Closest'
                    found_mat = mat
                    found_tex = node

        node_tree = found_mat.node_tree
        nodes = list(node_tree.nodes)
        for node in nodes:
            if node.type == 'OUTPUT_MATERIAL':
                found_output = node
            elif node.type != 'TEX_IMAGE':
                node_tree.nodes.remove(node)
        
        shadeless_group_node = get_shadeless_node(node_tree)
        node_tree.links.clear()
        node_tree.links.new(shadeless_group_node.inputs[0], found_tex.outputs[0])
        node_tree.links.new(found_output.inputs[0], shadeless_group_node.outputs[0])

        ## Select the mesh and zoom to it
        # 0 is the camera
        # context.view_layer.objects.active = bpy.data.objects[1]
        for area in context.screen.areas:
            if area.type == 'VIEW_3D':
                ctx = context.copy()
                ctx['area'] = area
                ctx['region'] = area.regions[-1]
                bpy.ops.view3d.view_selected(ctx)

                # while we're at it, update the shading mode
                for space in area.spaces: # iterate through spaces in current VIEW_3D area
                    if space.type == 'VIEW_3D': # check if space is a 3D view
                        space.shading.type = 'MATERIAL' # set the viewport shading to rendered
                        
