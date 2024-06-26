# MIT License
#
# Copyright (c) 2020 Tri Minh Cao
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Data structures for DEF Parser
Author: Tri Minh Cao
Email: tricao@utdallas.edu
Date: August 2016

Changes by Tobias Senti
- SPECIALNETS support
- Signal Class support for Nets and SpecialNets
- Write Format: Make similar to OpenROAD's DEF writer
- SOURCE support for Components
- FIXED and PLACED support for Components during writing
"""
from .util import *


class Pins:
    """
    Class Pins represents the PINS section in DEF file. It contains
    individual Pin objects.
    """

    def __init__(self, num_pins):
        self.type = "PINS_DEF"
        self.num_pins = num_pins
        self.pins = []
        self.pin_dict = {}

    def parse_next(self, info):
        if info[0] == "-":
            # create a new pin
            # print (info[1])
            current_pin = Pin(info[1])
            self.pins.append(current_pin)
            self.pin_dict[info[1]] = current_pin
            # print ("new")
        else:
            current_pin = self.get_last_pin()
            # print ("last")
            # parse the next info
            if info[0] == "NET":
                current_pin.net = info[1]
            elif info[0] == "DIRECTION":
                current_pin.direction = info[1]
            elif info[0] == "USE":
                current_pin.use = info[1]
            elif info[0] == "LAYER":
                new_layer = Layer(info[1])
                new_layer.points.append([int(info[3]), int(info[4])])
                new_layer.points.append([int(info[7]), int(info[8])])
                current_pin.layer = new_layer
            elif info[0] == "PLACED":
                current_pin.placed = [int(info[2]), int(info[3])]
                current_pin.orient = info[5]

    def __len__(self):
        return len(self.pins)

    def __iter__(self):
        return self.pins.__iter__()

    def __getitem__(self, pin_name):
        return self.get_pin(pin_name)

    def get_last_pin(self):
        return self.pins[-1]

    def to_def_format(self):
        s = ""
        s += "PINS" + " " + str(self.num_pins) + " ;\n"
        for each_pin in self.pins:
            # check if the each_pin has Layer and Placed != None
            s += each_pin.to_def_format() + "\n"
        s += "END PINS"
        return s

    def get_pin(self, pin_name):
        return self.pin_dict[pin_name]


class Pin:
    """
    Class Pin represents an individual pin defined in the DEF file.
    """

    def __init__(self, name):
        self.type = "PIN_DEF"
        self.name = name
        self.net = None
        self.direction = None
        self.use = None
        self.layer = None
        self.placed = None
        self.orient = None

    # add methods to add information to the Pin object
    def __str__(self):
        s = ""
        s += self.type + ": " + self.name + "\n"
        s += "    " + "Name: " + self.net + "\n"
        s += "    " + "Direction: " + self.direction + "\n"
        s += "    " + "Use: " + self.use + "\n"
        if self.layer:
            s += "    " + "Layer: " + str(self.layer) + "\n"
        if self.placed:
            s += "    " + "Placed: " + str(self.placed) + " " + self.orient + "\n"
        return s

    def to_def_format(self):
        # - N1 + NET N1 + DIRECTION INPUT + USE SIGNAL
        #  + LAYER metal2 ( -70 0 ) ( 70 140 )
        #  + PLACED ( 27930 0 ) N ;
        s = ""
        s += "    - " + self.name + " + NET " + self.net
        s += " + DIRECTION " + self.direction + " + USE " + self.use + "\n"
        s += "      + PORT\n"
        if self.layer:
            s += "        + " + self.layer.to_def_format() + "\n"
        if self.placed:
            s += "        + " + "PLACED " + "( " + str(self.placed[0]) + " "
            s += str(self.placed[1]) + " ) " + self.orient
        s += " ;"
        return s

    def get_metal_layer(self):
        return self.layer.name


class Layer:
    """
    Class Layer represents a layer defined inside a PIN object
    """

    def __init__(self, name):
        self.type = "LAYER_DEF"
        self.name = name
        self.points = []

    def __str__(self):
        s = ""
        s += self.name
        for pt in self.points:
            s += " " + str(pt)
        return s

    def to_def_format(self):
        s = ""
        s += "LAYER" + " " + self.name
        for pt in self.points:
            s += " ( " + str(pt[0]) + " " + str(pt[1]) + " )"
        return s


class Components:
    """
    Class Components represents the COMPONENTS section in the DEF file.
    """

    def __init__(self, num_comps):
        self.type = "COMPONENTS_DEF"
        self.num_comps = num_comps
        self.comps = []
        self.comp_dict = {}

    def parse_next(self, info):
        if info[0] == "-":
            new_comp = Component(info[1])
            new_comp.macro = info[2]
            self.comps.append(new_comp)
            self.comp_dict[info[1]] = new_comp
        else:
            current_comp = self.get_last_comp()
            # parse the next info
            if info[0] == "PLACED" or info[0] == "FIXED":
                current_comp.location_type = info[0]
                current_comp.location = [int(info[2]), int(info[3])]
                current_comp.orient = info[5]
            elif info[0] == "SOURCE":
                current_comp.source = info[1]

    def __len__(self):
        return len(self.comps)

    def __getitem__(self, comp_name):
        return self.get_comp(comp_name)

    def __iter__(self):
        return self.comps.__iter__()

    def get_last_comp(self):
        return self.comps[-1]

    def get_comp(self, comp_name):
        return self.comp_dict[comp_name]

    def to_def_format(self):
        s = ""
        s += "COMPONENTS" + " " + str(self.num_comps) + " ;\n"
        for each_comp in self.comps:
            s += each_comp.to_def_format() + "\n"
        s += "END COMPONENTS"
        return s


class Component:
    """
    Represents individual component inside the COMPONENTS section in the DEF
    file.
    """

    def __init__(self, name):
        self.type = "COMPONENT_DEF"
        self.name = name
        self.macro = ''
        self.location = [0, 0]
        self.location_type = ''
        self.orient = ''
        self.source = ''

    def get_macro(self):
        return self.macro

    def __str__(self):
        s = ""
        s += self.type + ": " + self.name + "\n"
        s += "    " + "Macro: " + self.macro + "\n"
        s += "    " + "Location Type: " + self.location_type + " Location: " + str(self.location) + " " + self.orient + "\n"
        s += "    " + "Source: " + self.source + "\n"
        return s

    def to_def_format(self):
        s = ""
        s += "- " + self.name + " " + self.macro
        if self.source is not None:
            s += " + SOURCE " + self.source
        s += " + " + self.location_type
        s += " ( " + str(self.location[0]) + " " + str(self.location[1]) + " ) "
        s += self.orient + " ;"
        return s

class SpecialNets:
    """
    Represents the section SPECIALNETS in the DEF file.
    """

    def __init__(self, num_specialnets):
        self.type = "SPECIALNETS_DEF"
        self.num_specialnets = num_specialnets
        self.specialnets = []
        self.specialnet_dict = {}

    def parse_next(self, info):
        # remember to check for "(" before using split_parentheses
        # if we see "(", then it means new component or new pin
        # another method is to check the type of the object, if it is a list
        # then we know it comes from parentheses
        info = split_parentheses(info)
        if info[0] == "-":
            specialnet_name = info[1]
            new_specialnet = SpecialNet(specialnet_name)
            self.specialnets.append(new_specialnet)
            self.specialnet_dict[specialnet_name] = new_specialnet
            current_specialnet = new_specialnet
            del info[:2]
            # The net may be followed by signal class
            if not info:
                return
        else:
            current_specialnet = self.get_last_specialnet()

        # parse next info
        if isinstance(info[0], list):
            for comp in info:
                current_specialnet.comp_pin.append(comp)
        elif info[0] == "USE":
            current_specialnet.signal_class = info[1]
        elif info[0] == "ROUTED" or info[0] == "NEW":
            new_shape = Shape()
            new_shape.layer = info[1]
            new_shape.width = info[2]
            current_specialnet.shapes.append(new_shape)
        elif info[0] == "SHAPE":
            current_shape = current_specialnet.shapes[-1]
            current_shape.shape_type = info[1]

            for idx in range(2, len(info)):
                if isinstance(info[idx], list):
                    # this is a point
                    parsed_pt = info[idx]
                    new_pt = []
                    for j in range(len(parsed_pt)):
                        # if we see "*", the new coordinate comes from last
                        #  point's coordinate
                        if parsed_pt[j] == "*":
                            last_pt = new_routed.get_last_pt()
                            new_coor = last_pt[j]
                            new_pt.append(new_coor)
                        else:
                            new_pt.append(int(parsed_pt[j]))
                    # add new_pt to the new_routed
                    current_shape.points.append(new_pt)
                else:
                    # this should be via end point
                    if info[idx] != ";":
                        current_shape.end_via = info[idx]
                    # the location of end_via is the last point in the route
                    current_shape.end_via_loc = current_shape.points[-1]

    def __iter__(self):
        return self.specialnets.__iter__()

    def __len__(self):
        return len(self.specialnets)

    def get_last_specialnet(self):
        return self.specialnets[-1]

    def to_def_format(self):
        s = ""
        s += "SPECIALNETS" + " " + str(self.num_specialnets) + " ;\n"
        for each_specialnet in self.specialnets:
            s += each_specialnet.to_def_format() + "\n"
        s += "END SPECIALNETS"
        return s

class SpecialNet:
    """
    Represents individual SpecialNet inside SPECIALNETS section.
    """

    def __init__(self, name):
        self.type = "SPECIALNET_DEF"
        self.name = name
        self.comp_pin = []
        self.shapes = []
        self.signal_class = None

    def __str__(self):
        s = ""
        s += self.type + ": " + self.name + " Signal Class: "
        s += self.signal_class
        s += "\n    " + "Comp/Pin: "
        for comp in self.comp_pin:
            s += " " + str(comp)
        s += "\n"
        s += "    " + "Shapes: " + "\n"
        for shape in self.shapes:
            s += "    " + "    " + str(shapes) + "\n"
        return s

    def to_def_format(self):
        s = ""
        s += "    - " + self.name
        for each_comp in self.comp_pin:
            # study each comp/pin
            # if it's a pin, check the Pin object layer (already parsed) -
            # but how can we check the Pin object layer?
            s += " ( " + " ".join(each_comp) + " )"
        s += " + USE " + self.signal_class
        if self.shapes:
            s += "\n      + ROUTED " + self.shapes[0].to_def_format()
            for i in range(1, len(self.shapes)):
                s += "\n      " + "NEW " + self.shapes[i].to_def_format()
        s += " ;"
        return s

class Shape:
    """
    Represents a SHAPE definition inside a SPECIALNET.
    """

    def __init__(self):
        self.type = "SHAPE_DEF"
        self.layer = None
        self.width = None
        self.shape_type = None
        self.points = []
        self.end_via = None
        self.end_via_loc = None

    def __str__(self):
        s = ""
        s += self.layer
        s += " "
        s += self.width
        s += " "
        s += self.shape_type
        for pt in self.points:
            s += " " + str(pt)
        if self.end_via != None:
            s += " " + self.end_via
        return s

    def get_last_pt(self):
        return self.points[-1]

    def get_layer(self):
        return self.layer

    def to_def_format(self):
        s = ""
        s += self.layer
        s += " "
        s += self.width
        s += " + SHAPE "
        s += self.shape_type
        for pt in self.points:
            s += " ("
            for coor in pt:
                s += " " + str(coor)
            s += " )"
        if self.end_via != None:
            s += " " + self.end_via
        return s

class Nets:
    """
    Represents the section NETS in the DEF file.
    """

    def __init__(self, num_nets):
        self.type = "NETS_DEF"
        self.num_nets = num_nets
        self.nets = []
        self.net_dict = {}

    def parse_next(self, info):
        # remember to check for "(" before using split_parentheses
        # if we see "(", then it means new component or new pin
        # another method is to check the type of the object, if it is a list
        # then we know it comes from parentheses
        info = split_parentheses(info)
        if info[0] == "-":
            net_name = info[1]
            new_net = Net(net_name)
            self.nets.append(new_net)
            self.net_dict[net_name] = new_net
            current_net = new_net
            del info[:2]
            # The net may be followed by components and signal class
            if not info:
                return
        else:
            current_net = self.get_last_net()

        # parse next info
        if isinstance(info[0], list):
            for comp in info:
                current_net.comp_pin.append(comp)
        elif info[0] == "USE":
            current_net.signal_class = info[1]
        elif info[0] == "ROUTED" or info[0] == "NEW":
            new_routed = Routed()
            new_routed.layer = info[1]
            # add points to the new_routed
            for idx in range(2, len(info)):
                if isinstance(info[idx], list):
                    # this is a point
                    parsed_pt = info[idx]
                    new_pt = []
                    for j in range(len(parsed_pt)):
                        # if we see "*", the new coordinate comes from last
                        #  point's coordinate
                        if parsed_pt[j] == "*":
                            last_pt = new_routed.get_last_pt()
                            new_coor = last_pt[j]
                            new_pt.append(new_coor)
                        else:
                            new_pt.append(int(parsed_pt[j]))
                    # add new_pt to the new_routed
                    new_routed.points.append(new_pt)
                else:
                    # this should be via end point
                    if info[idx] != ";":
                        new_routed.end_via = info[idx]
                    # the location of end_via is the last point in the route
                    new_routed.end_via_loc = new_routed.points[-1]
            # add new_routed to the current_net
            current_net.routed.append(new_routed)

    def __iter__(self):
        return self.nets.__iter__()

    def __len__(self):
        return len(self.nets)

    def get_last_net(self):
        return self.nets[-1]

    def to_def_format(self):
        s = ""
        s += "NETS" + " " + str(self.num_nets) + " ;\n"
        for each_net in self.nets:
            s += each_net.to_def_format() + "\n"
        s += "END NETS"
        return s


class Net:
    """
    Represents individual Net inside NETS section.
    """

    def __init__(self, name):
        self.type = "NET_DEF"
        self.name = name
        self.comp_pin = []
        self.routed = []
        self.signal_class = None

    def __str__(self):
        s = ""
        s += self.type + ": " + self.name + " Signal Class: "
        s += self.signal_class
        s += "\n    " + "Comp/Pin: "
        for comp in self.comp_pin:
            s += " " + str(comp)
        s += "\n"
        s += "    " + "Routed: " + "\n"
        for route in self.routed:
            s += "    " + "    " + str(route) + "\n"
        return s

    def to_def_format(self):
        s = ""
        s += "    - " + self.name
        for each_comp in self.comp_pin:
            # study each comp/pin
            # if it's a pin, check the Pin object layer (already parsed) -
            # but how can we check the Pin object layer?
            s += " ( " + " ".join(each_comp) + " )"
        s += " + USE " + self.signal_class
        if self.routed:
            s += "\n      + ROUTED " + self.routed[0].to_def_format()
            for i in range(1, len(self.routed)):
                s += "\n      " + "NEW " + self.routed[i].to_def_format()
        s += " ;"
        return s


class Routed:
    """
    Represents a ROUTED definition inside a NET.
    """

    def __init__(self):
        self.type = "ROUTED_DEF"
        self.layer = None
        self.points = []
        self.end_via = None
        self.end_via_loc = None

    def __str__(self):
        s = ""
        s += self.layer
        for pt in self.points:
            s += " " + str(pt)
        if self.end_via != None:
            s += " " + self.end_via
        return s

    def get_last_pt(self):
        return self.points[-1]

    def get_layer(self):
        return self.layer

    def to_def_format(self):
        s = ""
        s += self.layer
        previous_pt = None
        for pt in self.points:
            s += " ("
            for idx, coor in enumerate(pt):
                if previous_pt is not None and idx < len(previous_pt) and previous_pt[idx] == coor:
                    s += " *"
                else:
                    s += " " + str(coor)
            s += " )"
            previous_pt = pt
        if self.end_via != None:
            s += " " + self.end_via
        return s


class Tracks:
    """
    Represents a TRACKS definition inside the DEF file.
    """

    def __init__(self, name):
        self.type = "TRACKS_DEF"
        self.name = name
        self.pos = None
        self.do = None
        self.step = None
        self.layer = None

    def to_def_format(self):
        s = ""
        s += "TRACKS" + " " + self.name + " " + str(self.pos) + " "
        s += "DO" + " " + str(self.do) + " " + "STEP" + " " + str(self.step)
        s += " " + "LAYER" + " " + self.layer + " ;"
        return s

    def get_layer(self):
        return self.layer


class GCellGrid:
    """
    Represents a GCELLGRID definition in the DEF file.
    """

    def __init__(self, name):
        self.type = "GCELLGRID_DEF"
        self.name = name
        self.pos = None
        self.do = None
        self.step = None

    def to_def_format(self):
        s = ""
        s += "GCELLGRID" + " " + self.name + " " + str(self.pos) + " "
        s += "DO" + " " + str(self.do) + " " + "STEP" + " " + str(self.step)
        s += " ;"
        return s


class Row:
    """
    Represents a ROW definition in the DEF file.
    """

    def __init__(self, name):
        self.type = "ROW_DEF"
        self.name = name
        self.site = None
        self.pos = None
        self.orient = None
        self.do = None
        self.by = None
        self.step = None

    def to_def_format(self):
        s = ""
        s += "ROW" + " " + self.name + " " + self.site + " "
        s += str(self.pos[0]) + " " + str(self.pos[1]) + " " + self.orient + " "
        s += "DO" + " " + str(self.do) + " " + "BY" + " " + str(self.by) + " "
        s += "STEP" + " " + str(self.step[0]) + " " + str(self.step[1])
        s += " ;"
        return s


class Property:
    """
    Represents a PROPERTYDEFINITIONS in the DEF file.
    """

    def __init__(self):
        self.type = "PROPERTY_DEF"
        self.texts = []

    def parse_next(self, info):
        new_line = " ".join(info)
        self.texts.append(new_line)

    def to_def_format(self):
        s = ""
        s += "PROPERTYDEFINITIONS\n"
        for each_line in self.texts:
            s += "    " + each_line + "\n"
        s += "END PROPERTYDEFINITIONS\n"
        return s
