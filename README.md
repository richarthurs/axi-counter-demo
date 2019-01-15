## AXI Counter Demo
An AXI-enabled counter with several interesting features. 


## Project Setup
1. Clone this repo
2. Open Vivado and open the project `.xpr` file in this repo. Don't worry about the warning messages that pop up. 
3. In the Vivado TCL console, `cd fpga/axi-counter-demo`
4. In the Vivado TCL console, `source axi-counter-demo-setup.tcl`

Wait a few seconds for the project to be re-created. 

5. Right click the entry under Sources/Design sources/ and choose `Generate HDL Wrapper` in the block diagram view. Let Vivado manage it automatically. 
6. Generate bitstream
