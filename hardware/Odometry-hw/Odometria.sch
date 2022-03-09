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
<text x="73.66" y="38.1" size="1.778" layer="94">Disco Magnetico</text>
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
<wire x1="40.64" y1="50.8" x2="40.64" y2="55.88" width="0.1524" layer="94"/>
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
<wire x1="45.72" y1="25.4" x2="55.88" y2="25.4" width="0.1524" layer="94"/>
<wire x1="55.88" y1="25.4" x2="66.04" y2="25.4" width="0.1524" layer="94"/>
<wire x1="66.04" y1="25.4" x2="66.04" y2="50.8" width="0.1524" layer="94"/>
<wire x1="66.04" y1="50.8" x2="66.04" y2="55.88" width="0.1524" layer="94"/>
<wire x1="66.04" y1="55.88" x2="55.88" y2="55.88" width="0.1524" layer="94"/>
<wire x1="55.88" y1="55.88" x2="45.72" y2="55.88" width="0.1524" layer="94"/>
<wire x1="71.12" y1="43.18" x2="71.12" y2="35.56" width="0.1524" layer="94"/>
<wire x1="71.12" y1="35.56" x2="99.06" y2="35.56" width="0.1524" layer="94"/>
<wire x1="99.06" y1="35.56" x2="99.06" y2="43.18" width="0.1524" layer="94"/>
<wire x1="99.06" y1="43.18" x2="83.82" y2="43.18" width="0.1524" layer="94"/>
<text x="48.26" y="45.72" size="1.778" layer="91">ATMEGA328</text>
<text x="48.26" y="40.64" size="1.778" layer="91">ATMEGA(USB)</text>
<text x="25.4" y="60.96" size="1.778" layer="91">MCP2515</text>
<text x="76.2" y="50.8" size="1.778" layer="94">Sensor/Sensores</text>
<wire x1="83.82" y1="43.18" x2="71.12" y2="43.18" width="0.1524" layer="94"/>
<wire x1="71.12" y1="55.88" x2="71.12" y2="50.8" width="0.1524" layer="94"/>
<wire x1="71.12" y1="50.8" x2="71.12" y2="48.26" width="0.1524" layer="94"/>
<wire x1="71.12" y1="48.26" x2="83.82" y2="48.26" width="0.1524" layer="94"/>
<wire x1="83.82" y1="48.26" x2="99.06" y2="48.26" width="0.1524" layer="94"/>
<wire x1="99.06" y1="48.26" x2="99.06" y2="55.88" width="0.1524" layer="94"/>
<wire x1="99.06" y1="55.88" x2="71.12" y2="55.88" width="0.1524" layer="94"/>
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
<wire x1="66.04" y1="50.8" x2="71.12" y2="50.8" width="0.1524" layer="94"/>
<wire x1="83.82" y1="48.26" x2="83.82" y2="43.18" width="0.1524" layer="94"/>
<wire x1="127" y1="60.96" x2="127" y2="40.64" width="0.1524" layer="94"/>
<wire x1="127" y1="40.64" x2="125.73" y2="40.64" width="0.1524" layer="94"/>
<wire x1="125.73" y1="40.64" x2="125.73" y2="60.96" width="0.1524" layer="94"/>
<wire x1="125.73" y1="60.96" x2="127" y2="60.96" width="0.1524" layer="94"/>
<wire x1="127" y1="54.61" x2="128.27" y2="54.61" width="0.1524" layer="95"/>
<wire x1="128.27" y1="54.61" x2="128.27" y2="44.45" width="0.1524" layer="95"/>
<wire x1="128.27" y1="44.45" x2="127" y2="44.45" width="0.1524" layer="95"/>
<wire x1="127" y1="59.69" x2="127" y2="58.42" width="0.1524" layer="95"/>
<wire x1="127" y1="58.42" x2="128.27" y2="58.42" width="0.1524" layer="95"/>
<wire x1="128.27" y1="58.42" x2="128.27" y2="59.69" width="0.1524" layer="95"/>
<wire x1="128.27" y1="59.69" x2="127" y2="59.69" width="0.1524" layer="95"/>
<wire x1="125.73" y1="58.42" x2="124.46" y2="58.42" width="0.1524" layer="95"/>
<wire x1="124.46" y1="58.42" x2="124.46" y2="52.07" width="0.1524" layer="95"/>
<wire x1="124.46" y1="52.07" x2="125.73" y2="52.07" width="0.1524" layer="95"/>
<wire x1="125.73" y1="46.99" x2="124.46" y2="46.99" width="0.1524" layer="95"/>
<wire x1="124.46" y1="46.99" x2="124.46" y2="43.18" width="0.1524" layer="95"/>
<wire x1="124.46" y1="43.18" x2="125.73" y2="43.18" width="0.1524" layer="95"/>
<wire x1="132.08" y1="50.8" x2="132.08" y2="48.26" width="0.1524" layer="95"/>
<wire x1="132.08" y1="50.8" x2="149.86" y2="50.8" width="0.1524" layer="95"/>
<wire x1="149.86" y1="50.8" x2="149.86" y2="48.26" width="0.1524" layer="95"/>
<wire x1="149.86" y1="48.26" x2="132.08" y2="48.26" width="0.1524" layer="95"/>
<wire x1="139.7" y1="58.42" x2="139.7" y2="40.64" width="0.1524" layer="94"/>
<wire x1="139.7" y1="40.64" x2="144.78" y2="40.64" width="0.1524" layer="94"/>
<wire x1="144.78" y1="40.64" x2="144.78" y2="58.42" width="0.1524" layer="94"/>
<wire x1="144.78" y1="58.42" x2="139.7" y2="58.42" width="0.1524" layer="94"/>
<wire x1="139.7" y1="58.42" x2="144.78" y2="58.42" width="0.1524" layer="95"/>
<wire x1="144.78" y1="58.42" x2="144.78" y2="60.96" width="0.1524" layer="95"/>
<wire x1="144.78" y1="60.96" x2="139.7" y2="60.96" width="0.1524" layer="95"/>
<wire x1="139.7" y1="60.96" x2="139.7" y2="58.42" width="0.1524" layer="95"/>
<wire x1="139.7" y1="60.96" x2="144.78" y2="66.04" width="0.1524" layer="95"/>
<wire x1="144.78" y1="66.04" x2="139.7" y2="66.04" width="0.1524" layer="95"/>
<wire x1="139.7" y1="66.04" x2="144.78" y2="71.12" width="0.1524" layer="95"/>
<wire x1="144.78" y1="71.12" x2="139.7" y2="71.12" width="0.1524" layer="95"/>
<wire x1="139.7" y1="71.12" x2="144.78" y2="76.2" width="0.1524" layer="95"/>
<wire x1="144.78" y1="76.2" x2="139.7" y2="76.2" width="0.1524" layer="95"/>
<wire x1="139.7" y1="76.2" x2="139.7" y2="78.74" width="0.1524" layer="95"/>
<wire x1="139.7" y1="78.74" x2="144.78" y2="78.74" width="0.1524" layer="95"/>
<wire x1="144.78" y1="78.74" x2="144.78" y2="76.2" width="0.1524" layer="95"/>
<text x="68.58" y="58.42" size="1.778" layer="91">https://www.pololu.com/product/2598</text>
<text x="68.58" y="63.5" size="1.778" layer="91">MA800GQ-P </text>
<text x="68.58" y="68.58" size="1.778" layer="91">https://reprap.org/wiki/Magnetic_Rotary_Encoder_v1.0</text>
<text x="68.58" y="73.66" size="1.778" layer="91">https://ams.com/nse-5310</text>
<text x="68.58" y="78.74" size="1.778" layer="91">https://www.instructables.com/id/Hack-your-servo-v200-Add-10-bit-incremental-a/</text>
<text x="68.58" y="83.82" size="1.778" layer="91">https://www.rls.si/en/am4096-12-bit-rotary-magnetic-encoder-chip</text>
<text x="25.4" y="15.24" size="1.778" layer="94">LED</text>
<wire x1="20.32" y1="20.32" x2="40.64" y2="20.32" width="0.1524" layer="94"/>
<wire x1="40.64" y1="20.32" x2="40.64" y2="15.24" width="0.1524" layer="94"/>
<wire x1="40.64" y1="15.24" x2="40.64" y2="10.16" width="0.1524" layer="94"/>
<wire x1="40.64" y1="10.16" x2="20.32" y2="10.16" width="0.1524" layer="94"/>
<wire x1="20.32" y1="10.16" x2="20.32" y2="20.32" width="0.1524" layer="94"/>
<wire x1="40.64" y1="15.24" x2="55.88" y2="15.24" width="0.1524" layer="94"/>
<wire x1="55.88" y1="15.24" x2="55.88" y2="25.4" width="0.1524" layer="94"/>
<text x="43.18" y="20.32" size="1.778" layer="91">Canal PWM</text>
</plain>
<instances>
</instances>
<busses>
<bus name="B$1">
<segment>
<wire x1="149.86" y1="68.58" x2="149.86" y2="33.02" width="0.1524" layer="92"/>
<wire x1="149.86" y1="33.02" x2="152.4" y2="33.02" width="0.1524" layer="92"/>
<wire x1="152.4" y1="33.02" x2="152.4" y2="68.58" width="0.1524" layer="92"/>
<wire x1="152.4" y1="68.58" x2="149.86" y2="68.58" width="0.1524" layer="92"/>
</segment>
</bus>
<bus name="B$2">
<segment>
<wire x1="127" y1="58.42" x2="132.08" y2="58.42" width="0.1524" layer="92"/>
<wire x1="132.08" y1="58.42" x2="132.08" y2="55.88" width="0.1524" layer="92"/>
<wire x1="132.08" y1="55.88" x2="127" y2="55.88" width="0.1524" layer="92"/>
</segment>
</bus>
<bus name="B$3">
<segment>
<wire x1="132.08" y1="43.18" x2="132.08" y2="40.64" width="0.1524" layer="92"/>
<wire x1="132.08" y1="40.64" x2="127" y2="40.64" width="0.1524" layer="92"/>
<wire x1="127" y1="40.64" x2="127" y2="43.18" width="0.1524" layer="92"/>
<wire x1="127" y1="43.18" x2="132.08" y2="43.18" width="0.1524" layer="92"/>
</segment>
</bus>
</busses>
<nets>
<net name="N$1" class="0">
<segment>
<wire x1="129.54" y1="55.88" x2="134.62" y2="55.88" width="0.1524" layer="91"/>
<wire x1="134.62" y1="55.88" x2="134.62" y2="43.18" width="0.1524" layer="91"/>
<wire x1="134.62" y1="43.18" x2="129.54" y2="43.18" width="0.1524" layer="91"/>
<wire x1="129.54" y1="43.18" x2="129.54" y2="55.88" width="0.1524" layer="91"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
