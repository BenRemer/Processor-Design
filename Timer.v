module Timer(ABUS,DBUS,WE,INTR,CLK,LOCK,INIT,DEBUG);  
parameter BITS;
parameter BASE;
... 

input wire [(BITS-1):0] ABUS;
inout wire [(BITS-1):0] DBUS;
input wire WE,CLK,LOCK,INIT;
output wire INTR;
...
wire selCtl=(ABUS==BASE);
wire wrCtl=WE&&selCtl;
wire rdCtl=(!WE)&&selCtl;
...
assign DBUS=rdCtl?{...Contents of the TCTL register...}:
            rdCnt?{...Contents of the TCNT register...}:
            {BITS{1'bz}};