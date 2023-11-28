# Generates Library

from footprint import Footprint

import json
from mako.template import Template
from kiutils.items.common import Position as KiPosition
from kiutils.footprint import Footprint as KiFootprint
from kiutils.footprint import DrillDefinition
from kiutils.footprint import Pad as KiPad
from kiutils.items.fpitems import FpRect

config_file_name = "./config/pdk.json"
tech_file_name = "./config/technology.json"
liberty_prefix = "liberty74_"
lef_name = "liberty74"
pdk_path = "./pdk/"
openroad_path = pdk_path + "openroad/"
kicad_path = pdk_path + "kicad/"
footprint_path = kicad_path + "footprints/"
lib_path = pdk_path + "lib/"
lef_path = pdk_path + "lef/"
verilog_path = pdk_path + "verilog/"

# Load Technology JSON
tech_file = open(tech_file_name)
tech_json = json.load(tech_file)

# Read Footprints
footprints = {}
for footprint_data in tech_json["footprints"]:
    for name in footprint_data["names"]:
        footprints[name] = footprint_data

# Technology Info
technology = tech_json["technology"]
if 'row_height_multiplier' in technology:
    technology['row_height'] = technology['row_height_multiplier'] * technology['y_wire_pitch']

tech_file.close()

new_fps = {}

# Generate Footprints
for fp in footprints:
    print(f"Generating footprint {fp}")

    new_fps.update(Footprint.from_json(footprints[fp], technology))

    footprints[fp]["pin_lef_template"] = [""]
    footprints[fp]["power_pin_lef_template"] = [""]
    
    footprints[fp]["cell_height"] = new_fps[fp].get_cell_height()
    footprints[fp]["cell_width"] = new_fps[fp].get_cell_width()

    # Loop over pins
    for i in range(1, footprints[fp]["num_pins"] + 1):
        footprints[fp]["pin_lef_template"].append(new_fps[fp].get_pin_lef(i))
        footprints[fp]["power_pin_lef_template"].append(new_fps[fp].get_power_pin_lef(i))

print("Footprints generated")

# Load Tech LEF template
tech_lef_template = Template(filename="./templates/tech_lef.template")

tech_lef_context = {
    "technology": technology
}

print("Generating Tech LEF...")

rendered_tech_lef = tech_lef_template.render(**tech_lef_context)

with open(lef_path + lef_name + "_tech.lef", 'w', encoding='utf-8') as tech_lef_file:
    tech_lef_file.write(rendered_tech_lef)
    
# Load JSON file
config_file = open(config_file_name)
config_json = json.load(config_file)

# Load Corners
corners = {}
for corner_data in config_json["corners"]:
    corners[corner_data["name"]] = corner_data

# Load Corners
cells = config_json["cells"]

config_file.close()

# Load liberty template
lib_template = Template(filename="./templates/liberty.lib.template")

# Genereate Liberty Libraries
for c in corners:
    lib_name = liberty_prefix + c

    print(f"Generating {lib_name}...")

    lib_context = {
        "lib_name": lib_name,
        "corner": corners[c],
        "cells": cells,
        "footprints": footprints,
        "new_fps": new_fps
    }

    rendered_lib = lib_template.render(**lib_context)

    with open(lib_path + lib_name + ".lib", 'w', encoding='utf-8') as lib_file:
        lib_file.write(rendered_lib)
    
# Load LEF template
lef_template = Template(filename="./templates/lef.template")

lef_context = {
    "footprints": footprints,
    "cells": cells,
    "site_width": technology["x_wire_pitch"],
    "row_height": technology["row_height"]
}

print("Generating LEF...")

rendered_lef = lef_template.render(**lef_context)

with open(lef_path + lef_name + ".lef", 'w', encoding='utf-8') as lef_file:
    lef_file.write(rendered_lef)
    
# Load Verilog template
verilog_template = Template(filename="./templates/verilog.template")

verilog_context = {
    "cells": cells,
    "pwr_pins": False
}

print("Generating Verilog...")

rendered_verilog = verilog_template.render(**verilog_context)

with open(verilog_path + "verilog.sv", 'w', encoding='utf-8') as verilog_file:
    verilog_file.write(rendered_verilog)

verilog_context = {
    "cells": cells,
    "pwr_pins": True
}

print("Generating Verilog with power pins...")

rendered_verilog = verilog_template.render(**verilog_context)

with open(verilog_path + "verilog_pwr_pins.sv", 'w', encoding='utf-8') as verilog_file:
    verilog_file.write(rendered_verilog)
    
# Load tcl template
make_tracks_template = Template(filename="./templates/make_tracks.tcl.template")

tech_context = {
    "technology": technology
}

print("Generating make_tracks.tcl...")

rendered_make_tracks = make_tracks_template.render(**tech_context)

with open(openroad_path + "make_tracks.tcl", 'w', encoding='utf-8') as tcl_file:
    tcl_file.write(rendered_make_tracks)

print("Generating Kicad Footprints")

pcb_copper_layers = []
for i in range(0, tech_json['pcb_stackup']['copper_layers']):
    layer_name = ''
    ordinal = i
    if i == 0:
        layer_name = 'F'
    elif i == tech_json['pcb_stackup']['copper_layers'] - 1:
        layer_name = 'B'
        ordinal = 31
    else:
        layer_name = 'In' + str(i)
    
    layer_name += '.Cu'

    pcb_copper_layers.append(layer_name)

pcb_pad_layers = pcb_copper_layers
pcb_pad_layers.append('F.Paste')
pcb_pad_layers.append('F.Mask')

drill = DrillDefinition(
    oval = False,
    diameter = technology['via_diameter'],
    width = 0,
    offset = KiPosition(0, 0)
)
via_size = KiPosition(2 * technology['via_annular_ring'] + technology['via_diameter'], 2 * technology['via_annular_ring'] + technology['via_diameter'])

for cell in cells:
    fp = new_fps[cell['footprint']]

    kifp = KiFootprint().create_new(library_id = "liberty74:" + cell['name'] + '_N', value = cell['name'], type = 'smd')
    kifp.generator = 'liberty74'
    kifp.description = cell['desc']
    kifp.tags = cell['name']

    # Outline
    kifp.graphicItems.append(
        FpRect(
            start = KiPosition(X = 0, Y = 0),
            end = KiPosition(fp.get_cell_width(), -fp.get_cell_height()),
            layer = 'F.CrtYd',
            width = 0.05
        )
    )
    kifp.graphicItems.append(
        FpRect(
            start = KiPosition(X = 0, Y = 0),
            end = KiPosition(fp.get_cell_width(), -fp.get_cell_height()),
            layer = 'F.SilkS',
            width = 0.05
        )
    )
    kifp.graphicItems.append(
        FpRect(
            start = KiPosition(X = 0, Y = 0),
            end = KiPosition(fp.get_cell_width(), -fp.get_cell_height()),
            layer = 'B.SilkS',
            width = 0.05
        )
    )

    # Power Pins -> Tie Pins not handled correctly :(
    for power_pin in cell['power']:
        pin_function = power_pin['connect_to_net'] if 'connect_to_net' in power_pin else power_pin['name'] 
        
        pin_number = power_pin['pin_number']
        pin_rect = fp.get_pin(pin_number)
        power_pin_rect = fp.get_power_pin(pin_number)

        # Normal Pad for soldering
        kifp.pads.append(
            KiPad(
                number = pin_number,
                type = 'smd',
                shape = 'rect',
                position = pin_rect.get_center_kiposition(),
                size = pin_rect.get_size_kiposition(),
                layers = ['F.Cu', 'F.Paste', 'F.Mask'],
                pinFunction = pin_function
            )
        )

        # Connection to power rails
        kifp.pads.append(
            KiPad(
                number = pin_number,
                type = 'smd',
                shape = 'rect',
                position = power_pin_rect.get_center_kiposition(),
                size = power_pin_rect.get_size_kiposition(),
                layers = ['F.Cu'],
                pinFunction = pin_function
            )
        )

    # Pins
    for pin in cell['inputs']:
        pin_number = pin['pin_number']
        pin_rect = fp.get_pin(pin_number)

        # Add SMD pad
        kifp.pads.append(
            KiPad(
                number = pin_number,
                type = 'smd',
                shape = 'rect',
                position = pin_rect.get_center_kiposition(),
                size = pin_rect.get_size_kiposition(),
                layers = ['F.Cu', 'F.Paste', 'F.Mask'] if fp.is_single_layer_footprint() else pcb_pad_layers,
                pinFunction = pin['name']
            )
        )

    for pin in cell['outputs']:
        pin_number = pin['pin_number']
        pin_rect = fp.get_pin(pin_number)

        # Add SMD pad
        kifp.pads.append(
            KiPad(
                number = pin_number,
                type = 'smd',
                shape = 'rect',
                position = pin_rect.get_center_kiposition(),
                size = pin_rect.get_size_kiposition(),
                layers = ['F.Cu', 'F.Paste', 'F.Mask'] if fp.is_single_layer_footprint() else pcb_pad_layers,
                pinFunction = pin['name']
            )
        )

#        if not fp['single_layer_footprint']:
#            # Add Via pad
#            via_position = KiPosition(pad_dim[0], -pad_dim[1])
#            via_position.X += pad_width / 2
#            if pin['pin_number'] % 2 == 0:
#                via_position.Y -= via_size.Y / 2
#            else:
#                via_position.Y -= pad_height - via_size.Y / 2

#            kifp.pads.append(
#                KiPad(
#                    number = pin['pin_number'],
#                    type = 'thru_hole',
#                    shape = 'circle',
#                    position = via_position,
#                    size = via_size,
#                    drill = drill,
#                    layers = pcb_copper_layers,
#                    pinFunction = pin['name']
#                )
#            )

    kifp.to_file(footprint_path + cell['name'] + '_N.kicad_mod')

    # Generate rotated footprint
    for pad in kifp.pads:
        pad.position.X = fp.get_cell_width() - pad.position.X
        pad.position.Y = -pad.position.Y - fp.get_cell_height()

    for item in kifp.graphicItems:
        if isinstance(item, FpRect):
            item.start.X = fp.get_cell_width() - item.start.X
            item.end.X   = fp.get_cell_width() - item.end.X
            item.start.Y = -item.start.Y - fp.get_cell_height()
            item.end.Y   = -item.end.Y   - fp.get_cell_height()

    kifp.entryName = cell['name'] + '_S'

    kifp.to_file(footprint_path + cell['name'] + '_S.kicad_mod')
    
print("Done!")
