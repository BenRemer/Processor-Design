1. Try implementing the basic pipeline for addi, sw and br.
2. Change code snippet. 
	parameter IMEMINITFILE = "tests/test2.mif";

3. Uncomment code snippet. 
	initial begin
    		$readmemh("tests/test2.hex", imem);
  	end

4. Start simulation as last in class assignment.
5. Monitor PC_FE. If the implementation is right, you should see PC_FE interate from 0x100 - 0x120 as show in the png.