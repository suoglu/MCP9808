/* ------------------------------------------------ *
 * Title       : MCP9808 Interface                  *
 * Project     : MCP9808 Interface                  *
 * ------------------------------------------------ *
 * File        : mcp9808.v                          *
 * Author      : Yigit Suoglu                       *
 * Last Edit   : //2021                         *
 * ------------------------------------------------ *
 * Description : Interface module for MCP9808       *
 *               temperature sensor                 *
 * ------------------------------------------------ *
 * Revisions                                        *
 *     v1      : Inital version                     *
 * ------------------------------------------------ */
// ! use MCP4725 as template 
module mcp9808#(
  parameter SLAVE_ADD = 3'd000)( // ? address pins to input port
  input clk,
  input rst,
  input clkI2Cx2, //TODO
  //I2C pins
  inout SCL/* synthesis keep = 1 */,
  inout SDA/* synthesis keep = 1 */,
  //GPIO
  output [10:0] T_o, //TODO
  input [10:0] T_i, //TODO
  input [1:0] T_write, //TODO
  input [1:0] res_i, //TODO
  input shutdown, //?: Use a register to keep it stable  
  input update, //TODO
  output ready); //TODO
  //Module registers
  localparam CONFIG_REG = 4'b0001, //Configuration Register
            T_UPPER_REG = 4'b0010, //Alert Temperature Upper Boundary Trip register
            T_LOWER_REG = 4'b0011, //Alert Temperature Lower Boundary Trip register
             T_CRIT_REG = 4'b0100, //Critical Temperature Trip register
               TEMP_REG = 4'b0101, //Temperature register
             MAN_ID_REG = 4'b0110, //Manufacturer ID registe
             DEV_ID_REG = 4'b0111, //Device ID/Revision register
            RESOLTN_REG = 4'b1000; //Resolution register
  localparam I2C_ADDR = {4'b0011, SLAVE_ADD};
  localparam I2C_READY = 3'b000,
             I2C_START = 3'b001,
             I2C_ADDRS = 3'b011,
             I2C_WRITE = 3'b110,
         I2C_WRITE_ACK = 3'b010,
              I2C_READ = 3'b111,
          I2C_READ_ACK = 3'b101,
              I2C_STOP = 3'b100;
  //I2C state & I2C state control
  reg [2:0] I2C_state; //TODO
  wire I2C_done; //TODO
  wire I2CinReady, I2CinStart, I2CinAddrs, I2CinWrite, I2CinWriteAck, I2CinRead, I2CinReadAck, I2CinStop, I2CinAck;
  //Module State
  localparam SHUTDOWN = 3'b110,
                 IDLE = 3'b000,
               CONFIG = 3'b100,
               CH_RES = 3'b011,
            READ_TEMP = 3'b010,
          SET_T_BOUND = 3'b001;     
  reg [2:0] state;
  wire chRes; //Change resolution 
  //Temperature Register write control
  localparam T_CRIT = 2'b11,
             T_UPPR = 2'b10,
             T_LOWR = 2'b01,
               NO_T = 2'b00;
  wire writeTemp;
  //Transmisson counters
  reg [1:0] byteCounter;
  reg [2:0] bitCounter;
  reg byteCountDone; //TODO
  wire bitCountDone;

  reg I2C_busy; //Another master is using I2C

  //Generate I2C signals with tri-state
  reg SCLK; //Internal I2C clock, always thicks
  wire SCL_claim; //TODO
  wire SDA_claim; //TODO
  wire SDA_write; //TODO

  //Local config
  reg [1:0] res; //TODO

  //Delay I2C signals, finding edges and conditions
  reg SDA_d;
  wire SDA_posedge, SDA_negedge;
  wire I2C_startCond, I2C_stopCond;

  //Decode I2C state
  assign I2CinReady = (I2C_state == I2C_READY);
  assign I2CinStart = (I2C_state == I2C_START);
  assign I2CinAddrs = (I2C_state == I2C_ADDRS);
  assign I2CinWrite = (I2C_state == I2C_WRITE);
  assign I2CinWriteAck = (I2C_state == I2C_WRITE_ACK);
  assign I2CinRead = (I2C_state == I2C_READ);
  assign I2CinReadAck = (I2C_state == I2C_READ_ACK);
  assign I2CinStop = (I2C_state == I2C_STOP);
  assign I2CinAck = I2CinWriteAck | I2CinReadAck;

  //State control
  assign writeTemp = (T_write != NO_T);
  assign chRes = (res != res_i);
  always@(posedge clk or posedge rst)
    begin
      if(rst)
        begin
          state <= IDLE;
        end
      else
        begin
          case(state)
            IDLE:
              begin
                if(shutdown)
                  state <= CONFIG;
                else if(writeTemp)
                  state <= SET_T_BOUND;
                else if(chRes)
                  state <= CH_RES;
                else if(update)
                  state <= READ_TEMP;
                else
                  state <= state;
              end
            SHUTDOWN:
              begin
                state <= (shutdown) ? state : CONFIG;
              end
            CONFIG:
              begin
                state <= (I2C_done) ? ((shutdown) ? SHUTDOWN : IDLE) : state;
              end
            CH_RES:
              begin
                state <= (I2C_done) ? IDLE : state;
              end
            READ_TEMP:
              begin
                state <= (I2C_done) ? IDLE : state;
              end
            SET_T_BOUND:
              begin
                state <= (I2C_done) ? IDLE : state;
              end
            default:
              begin
                state <= IDLE;
              end
          endcase
        end
    end

  //Tri-state control for I2C lines
  assign SCL = (SCL_claim) ?    SCLK   : 1'bZ;
  assign SDA = (SDA_claim) ? SDA_write : 1'bZ;

  //Delay I2C & Edge detect
  always @(posedge clk) 
    begin
      SDA_d <= SDA;
    end

  //Listen I2C Bus & cond. gen.
  assign SDA_negedge = ~SDA &  SDA_d;
  assign SDA_posedge =  SDA & ~SDA_d;
  assign I2C_startCond = SCL & SDA_negedge;
  assign I2C_stopCond  = SCL & SDA_posedge;
  //Determine if an other master is using the bus
  always@(posedge clk or posedge rst)
    begin
      if(rst)
        begin
          I2C_busy <= 1'b0;
        end
      else
        begin
          case(I2C_busy)
            1'b0:
              begin
                I2C_busy <= I2C_startCond;
              end
            1'b1:
              begin
                I2C_busy <= ~I2C_stopCond;
              end
          endcase
        end
    end
  
  //I2C byte counter
  always@(posedge I2CinAck or posedge I2CinStart)
    begin
      if(I2CinStart)
        begin
          byteCounter <= 2'd0;
        end
      else
        begin
          byteCounter <= byteCounter + 2'd1;
        end
    end

  //I2C bit counter
  assign bitCountDone = ~|bitCounter;
  always@(posedge SCL) 
    begin
      case(I2C_state)
        I2C_ADDR: bitCounter <= bitCounter + 3'd1;
        I2C_WRITE: bitCounter <= bitCounter + 3'd1;
        I2C_READ: bitCounter <= bitCounter + 3'd1;
        default: bitCounter <= 3'd0;
      endcase
    end

  //Divide clkI2Cx2 to get I2C clk
  always@(posedge clkI2Cx2 or posedge rst)
    begin
      if(rst)
        begin
          SCLK <= 1'b1;
        end
      else
        begin
          SCLK <= ~SCLK;
        end
    end
endmodule
