#!/usr/bin/env python
#http://wiki.ros.org/dynamic_reconfigure/Tutorials/HowToWriteYourFirstCfgFile
PACKAGE = "px4_planner"

from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()

gen.add("Hor-Vel", double_t,    0, "Hor-Vel", 2,  -5, 5)
gen.add("Rot-Vel", double_t, 0, "Rot-Vel",    2, -5,   5)
gen.add("Ver-Vel", double_t, 0, "Ver-Vel",    2, -5,   5)
gen.add("Altitude",    int_t,    0, "An Integer parameter", 0,  0, 10)

# size_enum = gen.enum([ gen.const("Small",      int_t, 0, "A small constant"),
#                        gen.const("Medium",     int_t, 1, "A medium constant"),
#                        gen.const("Large",      int_t, 2, "A large constant"),
#                        gen.const("ExtraLarge", int_t, 3, "An extra large constant")],
#                      "An enum to set size")

# gen.add("size", int_t, 0, "A size parameter which is edited via an enum", 1, 0, 3, edit_method=size_enum)

exit(gen.generate(PACKAGE, "px4-velocity", "planner"))
