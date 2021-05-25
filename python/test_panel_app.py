import math

import panel as pn
pn.extension()

range_slider = pn.widgets.RangeSlider(
    name='Range Slider', start=0, end=math.pi, value=(math.pi/4., math.pi/2.), step=0.01)
    
range_slider.param.watch(print, 'value')

row = pn.Row( range_slider)
row.servable()