/* ------------------------------------------------ *
 * Title       : Test board for MCP9808 Interface   *
 * Project     : MCP9808 Interface                  *
 * ------------------------------------------------ *
 * File        : testboard.v                        *
 * Author      : Yigit Suoglu                       *
 * Last Edit   : /04/2021                         *
 * ------------------------------------------------ *
 * Description : Verify MCP9808 Interface module    *
 * ------------------------------------------------ */

// `include "Test/ssd_util.v"
// `include "Test/btn_debouncer.v"
// `include "Test/clkGenParam.v"
// `include "Sources/mcp9808.v"

module board(
  input clk,
  input rst,
  //GPIO
  input btnR, //update
  input btnL, //write temp
  input shutdown, //sw15
  input [10:0] sw, //Temp in
  //I2C
  inout SCL/* synthesis keep = 1 */, //JB3
  inout SDA/* synthesis keep = 1 */, //JB4
  //GPIO
  output [3:0] an,
  output [6:0] seg,
  output ready); //JB1
  localparam addrsPins = 3'b0;
  //Internal uut wires
  wire [11:0] tempVal;//ssd dig [2:0]
  wire tempSign; //lsb sdd dig[3]
  wire [2:0] tempComp; //bits [3:1] of sdd dig[3]
  wire update; //btnR
  wire clkI2Cx2; //800kHz
  wire [10:0] tempInput;
  wire [1:0] tempWrite;
  wire [1:0] res;
  wire tempWReq;
  reg tempWReg;

  mcp9808 uut(clk,rst,clkI2Cx2,addrsPins,SCL,SDA,tempVal,tempSign,tempComp,tempInput,tempWrite,res,shutdown,update,ready);

  //I/O controllers & logic
  ssdController4 ssdcntr(clk, rst, 4'b1111, {tempComp,tempSign},tempVal[11:8], tempVal[7:4], tempVal[3:0], seg, an);
  debouncer db_R(clk, rst, btnR, update);
  debouncer db_L(clk, rst, btnL, tempWReq);
  
  assign tempInput = sw;

  assign tempWrite = (tempWReg) ? 2'b10 : 2'b00; //Upper : No write

  always@(posedge clk) //Regsiter temp write
    begin
      if(rst)
        tempWReg <= 1'b0;
      else
        case(tempWReg)
          1'b0:
            begin
              tempWReg <= tempWReq;
            end
          1'b1:
            begin
              tempWReg <= ~ready;
            end
        endcase
    end

  //Generate I2C clock
  clkGenP #(1250, 10) i2cClkGen(clk,rst,1'b1,clkI2Cx2);
endmodule
