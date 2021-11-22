<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="9.2.2">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="88" name="SimResults" color="9" fill="1" visible="yes" active="yes"/>
<layer number="89" name="SimProbes" color="9" fill="1" visible="yes" active="yes"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
</parts>
<sheets>
<sheet>
<plain>
<text x="25.4" y="50.8" size="1.778" layer="94">USB</text>
<text x="50.8" y="50.8" size="1.778" layer="94">MCU</text>
<text x="25.4" y="38.1" size="1.778" layer="94">VIN</text>
<text x="76.2" y="71.12" size="1.778" layer="94">SERVO</text>
<text x="25.4" y="63.5" size="1.778" layer="94">CAN BUS</text>
<wire x1="20.32" y1="68.58" x2="40.64" y2="68.58" width="0.1524" layer="94"/>
<wire x1="40.64" y1="68.58" x2="40.64" y2="63.5" width="0.1524" layer="94"/>
<wire x1="40.64" y1="63.5" x2="40.64" y2="58.42" width="0.1524" layer="94"/>
<wire x1="40.64" y1="58.42" x2="20.32" y2="58.42" width="0.1524" layer="94"/>
<wire x1="20.32" y1="58.42" x2="20.32" y2="68.58" width="0.1524" layer="94"/>
<wire x1="20.32" y1="55.88" x2="20.32" y2="48.26" width="0.1524" layer="94"/>
<wire x1="20.32" y1="48.26" x2="30.48" y2="48.26" width="0.1524" layer="94"/>
<wire x1="30.48" y1="48.26" x2="40.64" y2="48.26" width="0.1524" layer="94"/>
<wire x1="40.64" y1="48.26" x2="40.64" y2="50.8" width="0.1524" layer="94"/>
<wire x1="40.64" y1="50.8" x2="40.64" y2="53.34" width="0.1524" layer="94"/>
<wire x1="40.64" y1="53.34" x2="40.64" y2="55.88" width="0.1524" layer="94"/>
<wire x1="40.64" y1="55.88" x2="20.32" y2="55.88" width="0.1524" layer="94"/>
<wire x1="20.32" y1="43.18" x2="20.32" y2="35.56" width="0.1524" layer="94"/>
<wire x1="20.32" y1="35.56" x2="25.4" y2="35.56" width="0.1524" layer="94"/>
<wire x1="25.4" y1="35.56" x2="38.1" y2="35.56" width="0.1524" layer="94"/>
<wire x1="38.1" y1="35.56" x2="40.64" y2="35.56" width="0.1524" layer="94"/>
<wire x1="40.64" y1="35.56" x2="40.64" y2="43.18" width="0.1524" layer="94"/>
<wire x1="40.64" y1="43.18" x2="30.48" y2="43.18" width="0.1524" layer="94"/>
<wire x1="30.48" y1="43.18" x2="20.32" y2="43.18" width="0.1524" layer="94"/>
<wire x1="45.72" y1="55.88" x2="45.72" y2="50.8" width="0.1524" layer="94"/>
<wire x1="45.72" y1="50.8" x2="45.72" y2="25.4" width="0.1524" layer="94"/>
<wire x1="45.72" y1="25.4" x2="66.04" y2="25.4" width="0.1524" layer="94"/>
<wire x1="66.04" y1="25.4" x2="66.04" y2="55.88" width="0.1524" layer="94"/>
<wire x1="66.04" y1="55.88" x2="60.96" y2="55.88" width="0.1524" layer="94"/>
<wire x1="60.96" y1="55.88" x2="55.88" y2="55.88" width="0.1524" layer="94"/>
<wire x1="55.88" y1="55.88" x2="45.72" y2="55.88" width="0.1524" layer="94"/>
<wire x1="71.12" y1="76.2" x2="71.12" y2="73.66" width="0.1524" layer="94"/>
<wire x1="71.12" y1="73.66" x2="71.12" y2="71.12" width="0.1524" layer="94"/>
<wire x1="71.12" y1="71.12" x2="71.12" y2="68.58" width="0.1524" layer="94"/>
<wire x1="71.12" y1="68.58" x2="91.44" y2="68.58" width="0.1524" layer="94"/>
<wire x1="91.44" y1="68.58" x2="91.44" y2="76.2" width="0.1524" layer="94"/>
<wire x1="91.44" y1="76.2" x2="71.12" y2="76.2" width="0.1524" layer="94"/>
<text x="48.26" y="45.72" size="1.778" layer="91">ATMEGA328</text>
<text x="48.26" y="40.64" size="1.778" layer="91">ATMEGA(USB)</text>
<text x="25.4" y="60.96" size="1.778" layer="91">MCP2515</text>
<wire x1="30.48" y1="43.18" x2="30.48" y2="48.26" width="0.1524" layer="94"/>
<wire x1="40.64" y1="50.8" x2="45.72" y2="50.8" width="0.1524" layer="94"/>
<wire x1="40.64" y1="63.5" x2="55.88" y2="63.5" width="0.1524" layer="94"/>
<wire x1="55.88" y1="63.5" x2="55.88" y2="55.88" width="0.1524" layer="94"/>
<text x="22.86" y="27.94" size="1.778" layer="94">+5V</text>
<wire x1="20.32" y1="33.02" x2="20.32" y2="25.4" width="0.1524" layer="94"/>
<wire x1="20.32" y1="25.4" x2="30.48" y2="25.4" width="0.1524" layer="94"/>
<wire x1="30.48" y1="25.4" x2="30.48" y2="33.02" width="0.1524" layer="94"/>
<wire x1="30.48" y1="33.02" x2="25.4" y2="33.02" width="0.1524" layer="94"/>
<text x="35.56" y="27.94" size="1.778" layer="94">+3V3</text>
<wire x1="25.4" y1="33.02" x2="20.32" y2="33.02" width="0.1524" layer="94"/>
<wire x1="33.02" y1="33.02" x2="33.02" y2="25.4" width="0.1524" layer="94"/>
<wire x1="33.02" y1="25.4" x2="43.18" y2="25.4" width="0.1524" layer="94"/>
<wire x1="43.18" y1="25.4" x2="43.18" y2="33.02" width="0.1524" layer="94"/>
<wire x1="43.18" y1="33.02" x2="38.1" y2="33.02" width="0.1524" layer="94"/>
<wire x1="38.1" y1="33.02" x2="33.02" y2="33.02" width="0.1524" layer="94"/>
<wire x1="25.4" y1="35.56" x2="25.4" y2="33.02" width="0.1524" layer="94"/>
<wire x1="38.1" y1="35.56" x2="38.1" y2="33.02" width="0.1524" layer="94"/>
<wire x1="60.96" y1="55.88" x2="60.96" y2="71.12" width="0.1524" layer="94"/>
<wire x1="60.96" y1="71.12" x2="71.12" y2="71.12" width="0.1524" layer="94"/>
<text x="71.12" y="78.74" size="1.778" layer="91">https://hobbyking.com/es_es/hobbykingtm-hk15338-giant-digital-servo-mg-25kg-0-21sec-175g.html</text>
<text x="71.12" y="83.82" size="1.778" layer="91">https://hobbyking.com/es_es/turnigytm-s8166m-high-torque-servo-33kg-0-21sec-154g.html</text>
<text x="71.12" y="88.9" size="1.778" layer="91">https://hobbyking.com/es_es/hobbykingtm-hk15328a-analog-servo-bb-mg-12-8kg-0-20sec-58g.html</text>
<text x="20.32" y="15.24" size="1.778" layer="94">LED</text>
<wire x1="15.24" y1="20.32" x2="35.56" y2="20.32" width="0.1524" layer="94"/>
<wire x1="35.56" y1="20.32" x2="35.56" y2="15.24" width="0.1524" layer="94"/>
<wire x1="35.56" y1="15.24" x2="35.56" y2="10.16" width="0.1524" layer="94"/>
<wire x1="35.56" y1="10.16" x2="15.24" y2="10.16" width="0.1524" layer="94"/>
<wire x1="15.24" y1="10.16" x2="15.24" y2="20.32" width="0.1524" layer="94"/>
<wire x1="35.56" y1="15.24" x2="50.8" y2="15.24" width="0.1524" layer="94"/>
<wire x1="50.8" y1="15.24" x2="50.8" y2="25.4" width="0.1524" layer="94"/>
<text x="38.1" y="20.32" size="1.778" layer="91">PWM</text>
<text x="71.12" y="93.98" size="1.778" layer="91">Manechilla externa para casos de fallo de alimentación.</text>
<wire x1="20.32" y1="38.1" x2="15.24" y2="38.1" width="0.1524" layer="94"/>
<text x="-5.08" y="38.1" size="1.778" layer="94">CNN PEQUEÑO</text>
<wire x1="-7.62" y1="43.18" x2="-7.62" y2="35.56" width="0.1524" layer="94"/>
<wire x1="-7.62" y1="35.56" x2="15.24" y2="35.56" width="0.1524" layer="94"/>
<wire x1="15.24" y1="35.56" x2="15.24" y2="43.18" width="0.1524" layer="94"/>
<wire x1="15.24" y1="43.18" x2="-7.62" y2="43.18" width="0.1524" layer="94"/>
<text x="38.1" y="83.82" size="1.778" layer="94" rot="R180">BATERIA 2S
ELEV + CHG</text>
<wire x1="43.18" y1="78.74" x2="43.18" y2="81.28" width="0.1524" layer="94"/>
<wire x1="43.18" y1="81.28" x2="43.18" y2="86.36" width="0.1524" layer="94"/>
<wire x1="43.18" y1="86.36" x2="22.86" y2="86.36" width="0.1524" layer="94"/>
<wire x1="22.86" y1="86.36" x2="22.86" y2="78.74" width="0.1524" layer="94"/>
<wire x1="22.86" y1="78.74" x2="40.64" y2="78.74" width="0.1524" layer="94"/>
<wire x1="40.64" y1="78.74" x2="43.18" y2="78.74" width="0.1524" layer="94"/>
<wire x1="58.42" y1="81.28" x2="43.18" y2="81.28" width="0.1524" layer="94"/>
<wire x1="40.64" y1="53.34" x2="43.18" y2="53.34" width="0.1524" layer="94"/>
<wire x1="43.18" y1="53.34" x2="43.18" y2="76.2" width="0.1524" layer="94"/>
<wire x1="43.18" y1="76.2" x2="40.64" y2="76.2" width="0.1524" layer="94"/>
<wire x1="40.64" y1="76.2" x2="40.64" y2="78.74" width="0.1524" layer="94"/>
<wire x1="58.42" y1="81.28" x2="58.42" y2="73.66" width="0.1524" layer="94"/>
<wire x1="58.42" y1="73.66" x2="71.12" y2="73.66" width="0.1524" layer="94"/>
</plain>
<instances>
</instances>
<busses>
</busses>
<nets>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
<compatibility>
<note version="6.3" minversion="6.2.2" severity="warning">
Since Version 6.2.2 text objects can contain more than one line,
which will not be processed correctly with this version.
</note>
</compatibility>
</eagle>
