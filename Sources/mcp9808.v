/* ------------------------------------------------ *
 * Title       : MCP9808 Interface                  *
 * Project     : MCP9808 Interface                  *
 * ------------------------------------------------ *
 * File        : mcp9808.v                          *
 * Author      : Yigit Suoglu                       *
 * Last Edit   : 17/04/2021                         *
 * ------------------------------------------------ *
 * Description : Interface module for MCP9808       *
 *               temperature sensor                 *
 * ------------------------------------------------ *
 * Revisions                                        *
 *     v1      : Inital version                     *
 * ------------------------------------------------ */
module mcp9808(
  input clk,
  input rst,
  input clkI2Cx2,
  input [2:0] addressPins,
  //I2C pins
  inout SCL/* synthesis keep = 1 */,
  inout SDA/* synthesis keep = 1 */,
  //GPIO
  output [11:0] tempVal,
  output tempSign,
  output [2:0] tempComp,
  input [10:0] tempInput,
  input [1:0] tempWrite,
  input [1:0] res_i,
  input shutdown, //?: Use a register to keep it stable  
  input update,
  output ready);
  //Module registers
  localparam CONFIG_REG = 4'b0001, //Configuration Register
            T_UPPER_REG = 4'b0010, //Alert Temperature Upper Boundary Trip register
            T_LOWER_REG = 4'b0011, //Alert Temperature Lower Boundary Trip register
             T_CRIT_REG = 4'b0100, //Critical Temperature Trip register
               TEMP_REG = 4'b0101, //Temperature register
             MAN_ID_REG = 4'b0110, //Manufacturer ID registe
             DEV_ID_REG = 4'b0111, //Device ID/Revision register
            RESOLTN_REG = 4'b1000; //Resolution register
  localparam I2C_FADDR = 4'b0011; //fixed part of I2C address
  localparam I2C_READY = 3'b000,
             I2C_START = 3'b001,
             I2C_ADDRS = 3'b011,
             I2C_WRITE = 3'b110,
         I2C_WRITE_ACK = 3'b010,
              I2C_READ = 3'b111,
          I2C_READ_ACK = 3'b101,
              I2C_STOP = 3'b100;
  //I2C state & I2C state control
  reg [2:0] I2C_state;
  wire I2C_done;
  wire I2CinReady, I2CinStart, I2CinAddrs, I2CinWrite, I2CinWriteAck, I2CinRead, I2CinReadAck, I2CinStop, I2CinAck;
  wire readNwrite;
  wire [6:0] I2CAddress;
  //Module State
  localparam SHUTDOWN = 3'b110, //Steady state for shutdown
                 IDLE = 3'b000, //Steady state
               CONFIG = 3'b100, //Write config reg
               CH_RES = 3'b011, //Write res reg
             TEMP_PTR = 3'b001, //Write Amb Temp pointer for read
            READ_TEMP = 3'b010, //Read Amb Temp reg
          SET_T_BOUND = 3'b111; //Write Temp boundry reg  
  reg [2:0] state;
  wire inSHUTDOWN, inIDLE, inCONFIG, inCH_RES, inREAD_TEMP, inSET_T_BOUND, inTEMP_PRT;
  wire chRes; //Change resolution 
  //Temperature Register write control
  localparam T_CRIT = 2'b11,
             T_UPPR = 2'b10,
             T_LOWR = 2'b01,
               NO_T = 2'b00;
  wire writeTemp;
  //Transmisson counters
  reg [2:0] byteCounter;
  reg [2:0] bitCounter;
  reg byteCountDone;
  wire bitCountDone;

  reg I2C_busy; //Another master is using I2C

  //I2C data control
  reg [7:0] SDAsend, SDAsend_write;
  reg [15:0] SDAreceive;
  reg [15:0] temp;
  wire SDAshift, SDAupdate;

  //Generate I2C signals with tri-state
  reg SCLK; //Internal I2C clock, always thicks
  wire SCL_claim;
  wire SDA_claim;
  wire SDA_write;
  reg SDA_d_i2c;

  //Local config
  reg [1:0] res;

  //Signalling, change inputs only when set
  assign ready = I2CinRead & (inSHUTDOWN | inIDLE);

  //Get I2C address
  assign I2CAddress = {I2C_FADDR, addressPins};

  //Delay I2C signals, finding edges and conditions
  reg SDA_d;
  wire SDA_posedge, SDA_negedge;
  wire I2C_startCond, I2C_stopCond;

  //Decode I2C state
  assign     I2CinRead = (I2C_state == I2C_READ);
  assign     I2CinStop = (I2C_state == I2C_STOP);
  assign    I2CinReady = (I2C_state == I2C_READY);
  assign    I2CinStart = (I2C_state == I2C_START);
  assign    I2CinAddrs = (I2C_state == I2C_ADDRS);
  assign    I2CinWrite = (I2C_state == I2C_WRITE);
  assign  I2CinReadAck = (I2C_state == I2C_READ_ACK);
  assign I2CinWriteAck = (I2C_state == I2C_WRITE_ACK);
  assign      I2CinAck = I2CinWriteAck | I2CinReadAck;

  //State control
  assign writeTemp = (tempWrite != NO_T);
  assign chRes = (res != res_i);
  assign I2C_done = I2CinStop;
  assign readNwrite = inREAD_TEMP;
  always@(posedge clk or posedge rst) //state ch
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
                state <= (I2C_done) ? ((shutdown) ? SHUTDOWN : TEMP_PTR) : state;
              end
            CH_RES:
              begin
                state <= (I2C_done) ? TEMP_PTR : state;
              end
            TEMP_PTR:
              begin
                state <= (I2C_done) ? IDLE : state;
              end
            READ_TEMP:
              begin
                state <= (I2C_done) ? IDLE : state;
              end
            SET_T_BOUND:
              begin
                state <= (I2C_done) ? TEMP_PTR : state;
              end
            default:
              begin
                state <= IDLE;
              end
          endcase
        end
    end
    
  //Decode states
  assign        inIDLE = (state == IDLE);
  assign      inCONFIG = (state == CONFIG);
  assign      inCH_RES = (state == CH_RES);
  assign    inSHUTDOWN = (state == SHUTDOWN);
  assign    inTEMP_PRT = (state == TEMP_PTR);
  assign   inREAD_TEMP = (state == READ_TEMP);
  assign inSET_T_BOUND = (state == SET_T_BOUND);

  //Tri-state control for I2C lines
  assign SCL = (SCL_claim) ?    SCLK   : 1'bZ;
  assign SDA = (SDA_claim) ? SDA_write : 1'bZ;
  assign SCL_claim = ~I2CinReady;
  assign SDA_claim = I2CinStart | I2CinAddrs | I2CinWrite | I2CinReadAck | I2CinStop;
  assign SDA_write = (I2CinStart | I2CinReadAck | I2CinStop) ? 1'd0 : SDAsend[7];

  //Store resolution config, power on value 0x3
    always@(posedge inCH_RES or posedge rst)
      begin
        if(rst)
          begin
            res <= res_i;
          end
        else
          begin
            res <= res_i;
          end
      end

  //Ambiant temp handle
  assign {tempComp,tempSign,tempVal} = temp; //decode temp reg
  always@(posedge clk or posedge rst) //update temp reg from receive buffer
    begin
      if(rst)
        begin
          temp <= 16'h0;
        end
      else
        begin
          temp <= (I2CinStop & inREAD_TEMP) ? SDAreceive : temp;
        end
    end
  always@(posedge SCL) //fill receive buffer
    begin
      SDAreceive <= (I2CinRead) ? {SDAreceive[15:0], SDA} : SDAreceive;
    end

  //SDA handling
  always @(posedge clk) 
    begin
      SDA_d <= SDA;
    end
  always@(negedge clkI2Cx2)
    begin
      SDA_d_i2c <= SDA;
    end
  
  //SDA send buffer
  always@(negedge clkI2Cx2)
    begin
      if(SDAupdate)
        SDAsend <= SDAsend_write;
      else if(SDAshift & ~SCL & |bitCounter)
        SDAsend <= {SDAsend << 1};
    end
    assign SDAupdate = I2CinStart | I2CinWriteAck;
    assign SDAshift = I2CinAddrs | I2CinWrite;

  //Determine what to send
  always@*
    begin
      case(byteCounter)
        3'd0:
          begin
            SDAsend_write = {I2CAddress,readNwrite};
          end
        3'd1:
          case(state)
            CONFIG: SDAsend_write = {4'h0, CONFIG_REG};
            CH_RES: SDAsend_write = {4'h0, RESOLTN_REG};
            TEMP_PTR: SDAsend_write = {4'h0, TEMP_REG};
            SET_T_BOUND: 
              case(tempWrite)
                T_UPPR:
                  begin
                    SDAsend_write = {4'h0, T_UPPER_REG};
                  end
                T_LOWR:
                  begin
                    SDAsend_write = {4'h0, T_LOWER_REG};
                  end
                default:
                  begin
                    SDAsend_write = {4'h0, T_CRIT_REG};
                  end
              endcase
            default: SDAsend_write = 8'd0;
          endcase
        3'd2:
          case(state)
            CONFIG: SDAsend_write = {5'h0, 2'b0, shutdown};
            CH_RES: SDAsend_write = {6'h0, res};
            SET_T_BOUND: SDAsend_write = {3'h0, tempInput[10:6]};
            default: SDAsend_write = 8'd0;
          endcase
        3'd3:
          case(state)
            CONFIG: SDAsend_write = {4'h0,4'h0};
            SET_T_BOUND: SDAsend_write = {tempInput[5:0],2'h0};
            default: SDAsend_write = 8'd0;
          endcase
        default:
          SDAsend_write = 8'd0;
      endcase
    end

  //Listen I2C Bus & cond. gen.
  assign   SDA_negedge = ~SDA &  SDA_d;
  assign   SDA_posedge =  SDA & ~SDA_d;
  assign I2C_startCond =  SCL & SDA_negedge;
  assign I2C_stopCond  =  SCL & SDA_posedge;

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
                I2C_busy <= I2C_startCond & I2CinReady;
              end
            1'b1:
              begin
                I2C_busy <= ~I2C_stopCond & I2CinReady;
              end
          endcase
        end
    end
  
  //I2C state transactions
  always@(negedge clkI2Cx2 or posedge rst)
    begin
      if(rst)
        begin
          I2C_state <= I2C_READY;
        end
      else
        begin
          case(I2C_state)
            I2C_READY:
              begin
                I2C_state <= (~(inIDLE | inSHUTDOWN) & SCLK & ~I2C_busy) ? I2C_START : I2C_state;
              end
            I2C_START:
              begin
                I2C_state <= (~SCL) ? I2C_ADDRS : I2C_state;
              end
            I2C_ADDRS:
              begin
                I2C_state <= (~SCL & bitCountDone) ? I2C_WRITE_ACK : I2C_state;
              end
            I2C_WRITE_ACK:
              begin
                I2C_state <= (~SCL) ? ((~SDA_d_i2c & ~byteCountDone) ? ((~readNwrite) ? I2C_WRITE : I2C_READ): I2C_STOP) : I2C_state;
              end
            I2C_WRITE:
              begin
                I2C_state <= (~SCL & bitCountDone) ? I2C_WRITE_ACK : I2C_state;
              end
            I2C_READ:
              begin
                I2C_state <= (~SCL & bitCountDone) ? I2C_READ_ACK : I2C_state;
              end
            I2C_READ_ACK:
              begin
                I2C_state <= (~SCL) ? ((byteCountDone) ? I2C_STOP : I2C_READ) : I2C_state;
              end
            I2C_STOP:
              begin
                I2C_state <= (SCL) ? I2C_READY : I2C_state;
              end
          endcase
        end
    end
  
  //I2C byte counter
  always@(posedge I2CinAck or posedge I2CinStart)
    begin
      if(I2CinStart)
        begin
          byteCounter <= 3'd0;
        end
      else
        begin
          byteCounter <= byteCounter + 3'd1;
        end
    end
  
  //I2C byte counter done
  always@*
    begin
      case(state)
        CONFIG: byteCountDone = (byteCounter == 3'd4);
        CH_RES: byteCountDone = (byteCounter == 3'd3);
        TEMP_PTR: byteCountDone = (byteCounter == 3'd2);
        READ_TEMP: byteCountDone = (byteCounter == 3'd3);
        SET_T_BOUND: byteCountDone = (byteCounter == 3'd4);
        default: byteCountDone = 1'b1;
      endcase
    end

  //I2C bit counter
  assign bitCountDone = ~|bitCounter;
  always@(posedge SCL) 
    begin
      case(I2C_state)
        I2C_ADDRS: bitCounter <= bitCounter + 3'd1;
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
