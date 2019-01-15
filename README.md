## AXI Counter Demo
An AXI-enabled counter with several interesting features. 


## Project Setup
0. Ensure you have the Standard Setup described [here](https://github.com/richarthurs/rtl-ip)
1. Clone this repo underneath `~/fpga/`. For a final path of: `~/fpga/axi-counter-demo`
2. Open Vivado and expose the TCL console (there is a small button on the bottom left of the intro screen).
3. In the Vivado TCL console, `cd fpga/axi-counter-demo`
4. In the Vivado TCL console, `source setup.tcl`

Wait a few seconds for the project to be re-created. 
 
5. Generate bitstream. Don't worry about the critical warnings about negative clock skew, this is a known issue with the board setup and isn't a problem in practice. 


## Updating the Project in Git
If you make significant changes to the project itself (and not to IP coming in from a library), you may need to regenerate the setup script. 

1. In the TCL console: `cd fpga/axi-counter-demo`
2. In the TCL console: `write_project_tcl setup.tcl -force`
3. Inspect the TCL file header for any other files that need to be added to git. `git add` those file paths if any exist. 
4. Commit changes. 

## Pulling Updates
If you pull and see changes to the `setup.tcl` file, you need to run the following commands to rebuild the project from the TCL file. 

1. In the TCL console: `cd fpga/axi-counter-demo`
2. In the TCL console: `source setup.tcl`



