`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    output  wire                     awready,
    output  wire                     wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,
    output  wire                     arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  wire                     rvalid,
    output  wire [(pDATA_WIDTH-1):0] rdata,
        
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  wire                     ss_tready, 
    input   wire                     sm_tready, 
    output  wire                     sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  wire                     sm_tlast, 
    
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  wire [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);
/********************** configuration register **********************/
    reg [2:0] ap_state;       //bit 0: ap_start, bit 1: ap_done, bit 2: ap_idle
    reg [31:0] data_length;
    reg [31:0] tap_number;
/********************** AXI-LITE **********************/ 
    //axi ready and valid signal
    reg fir_awready;
    reg fir_wready;
    reg fir_arready;
    reg fir_rvalid;
    reg [31:0] fir_rdata;  
   
    assign awready = fir_awready;
    assign wready = fir_wready;
    assign arready = fir_arready;
    assign rvalid = fir_rvalid;
    assign rdata = fir_rdata; 
        
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            fir_awready <= 0;
            fir_wready <= 0;
            fir_arready <= 0;
            fir_rvalid <= 0;
        end else begin
            fir_awready <= (awvalid && wvalid) ? 1 : 0;
            fir_wready <= (awvalid && wvalid) ? 1 : 0;
            fir_arready <= (arvalid && rready) ? 1 : 0;
            fir_rvalid <= (arvalid && rready) ? 1 : 0;
        end
    end
    
    //tap ram
    assign tap_EN = (axis_rst_n) ? 1 : 0;
    assign tap_WE = (wvalid && awaddr[6])? 4'b1111 : 4'b0000; //0x40 for tap ram
    assign tap_A  = (awvalid && awaddr[6])? awaddr[5:0] : tap_A_read; //tap_A_read is host or fir engine read address
    assign tap_Di = (wvalid && awaddr[6])? wdata : 32'h00000000;
    
    //configuration write 
    always @(*) begin
        if(wvalid && !awaddr[6] && !awaddr[4]) ap_state[0] = wdata[0];//0x00 for ap_state (ap_start)
        else if(wvalid && !awaddr[6] && awaddr[4] && !awaddr[2]) data_length = wdata; //0x10 for data_length
        else if(wvalid && !awaddr[6] && awaddr[4] && awaddr[2]) tap_number = wdata; //0x14 for tap_number       
    end
    
    //configuration read
    always @(*) begin
        if (rready && araddr[6]) fir_rdata = tap_Do;
        else if (rready && !araddr[6] && !araddr[4]) fir_rdata[2:0] = ap_state[2:0];
        else if (rready && !araddr[6] && araddr[4] && !araddr[2]) fir_rdata = data_length;
        else if (rready && !araddr[6] && araddr[4] && araddr[2]) fir_rdata = tap_number;         
    end

/********************** FSM ap_state **********************/
    reg next_state;
    
    //ap_idle state transfer
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) ap_state[2] <= 1'b1;
        else ap_state[2] <= next_state;
    end 
    
    //combinational logic to generate next state from current state
    always @(*) begin
        case (ap_state[2])
            1'b1://idle
                if (ap_state[0] && ss_tvalid) next_state = 1'b0; //when ap_start programmed one, engine enters run mode
                else next_state = 1'b1;  
            1'b0://run
                if (sm_tvalid && sm_tlast) next_state = 1'b1; //when last data y is transferred, engine enters idle mode
                else next_state = 1'b0;
            default: next_state = 1'b1;
        endcase
    end
    
    //ap_start and ap_done logic of the engine
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) ap_state[1:0] <= 2'b00;  
        else begin
            case (next_state)
                1'b1://idle
                    if (!ap_state[1] && sm_tvalid && sm_tlast) ap_state[1] <= 1'b1; //last y is transferred
                    else if(ap_state[1] && rvalid && arvalid && !araddr[6] && !araddr[4]) ap_state[1] <= 1'b0; //ap_done is read
                    else ap_state[1] <= ap_state[1];
                1'b0://run
                    if (ap_state[0]) ap_state[0] <= 1'b0; //ap_start reset by engine
                    else ap_state[0] <= ap_state[0];
                default: ap_state[1:0] = ap_state[1:0];
            endcase
        end
    end

/********************** AXI_stream_X[n] **********************/         
    assign ss_tready = (ss_tvalid && !ap_state[2] && xn_count[3:0] == 4'd0) ? 1 : 0; //write data every 11 cycle
    assign data_EN = (axis_rst_n) ? 1 : 0; 
    assign data_WE = (ss_tready || (ap_state[2] && i_data_a != 6'd44)) ? 4'b1111 : 4'b0000;
    assign data_A = (ap_state[2]) ? i_data_a : fir_data_a;
    assign data_Di = (ap_state[2]) ? 0 : ss_tdata; //initialize data ram data to 0
    
    //initialize address
    reg  [5:0] i_data_a;
    
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if(!axis_rst_n) i_data_a <= -6'd04;
        else begin
            if (i_data_a == 6'd44) i_data_a <= 6'd44;
            else i_data_a <= i_data_a + 6'd04;
        end
    end
    
/********************** AXI_stream_Y[n] **********************/        
    reg sm_state;
    reg sm_next_state;
    reg [3:0] yn_count;
    reg [31:0] yn_total_count;
    
    assign sm_tvalid = (yn_count == 4'd1) ? 1 : 0;    
    assign sm_tdata  = (sm_tvalid) ? y_reg : 0;             
    assign sm_tlast  = (sm_tvalid && yn_total_count == data_length - 1)? 1 : 0; 
    
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n || ap_state[2]) yn_count <= 4'd0; 
        else begin
            if (yn_count == 4'd0) yn_count <= tap_number + 4'd2;
            else if (yn_count == 4'd1) yn_count <= tap_number;
            else yn_count <= yn_count - 1;
        end
    end
    
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) yn_total_count <= 32'd0;
        else begin
            if (sm_tvalid) begin
                if (yn_total_count == data_length)
                    yn_total_count <= data_length;
                else
                    yn_total_count <= yn_total_count + 1;
            end
            else yn_total_count <= yn_total_count;
        end
    end       

/********************** tap RAM Address Generator **********************/
    wire [5:0] tap_A_read;    
    reg  [5:0] fir_tap_A;
    
    assign tap_A_read = (ap_state[2] && arvalid)? araddr[5:0] : fir_tap_A; 

    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n || ap_state[2]) fir_tap_A <= 6'd00;
        else begin 
            if (fir_tap_A == (tap_number - 1)*4) fir_tap_A <= 6'd00;
            else  fir_tap_A <= fir_tap_A + 3'd4;
        end
    end
    
/********************** data RAM Address Generator **********************/
    reg [3:0] xn_count;
    reg [5:0] fir_data_aw;
    reg [5:0] fir_data_ar;
    wire [5:0] fir_data_a;
    
    assign fir_data_a = (ss_tready) ? fir_data_aw : fir_data_ar;
    
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n || ap_state[2]) xn_count <= 4'd0;      
        else begin
            if (xn_count == 4'd0) xn_count <= tap_number - 1;
            else xn_count <= xn_count - 1;
        end
    end
    
    //address write to data_ram
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n || ap_state[2]) fir_data_aw <= 6'd00;
        else begin
            if (xn_count == 4'd0) begin
                if (fir_data_aw == (tap_number - 1)*4) fir_data_aw <= 6'd00;
                else  fir_data_aw <= fir_data_aw + 6'd04;
            end else fir_data_aw <= fir_data_aw;
        end
    end
    
    //address read data_ram
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n || ap_state[2]) fir_data_ar <= (tap_number - 1)*4;
        else begin 
            if (xn_count != 4'd0) begin
                if (fir_data_ar == 6'd00) fir_data_ar <= (tap_number - 1)*4;
                else fir_data_ar <= fir_data_ar - 6'd04;
            end else fir_data_ar <= fir_data_ar;
        end
    end

/********************** FIR operation **********************/  
    reg [31:0] b_reg;
    reg [31:0] x_reg;
    reg [31:0] m_reg;
    reg  [31:0] y_reg;
       
    wire  [31:0] b;
    wire  [31:0] x;
    wire [31:0] y;
    wire  [31:0] m;
    
    assign b = (!tap_WE)? tap_Do : 32'd0;          
    assign x = (!data_WE)? data_Do : 32'd0;           
    assign m = b_reg * x_reg;           
    assign y = m_reg + y_reg;  
            
    //Pipeline Operation
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n || ap_state[2]) begin
            b_reg <= 32'd0;
            x_reg <= 32'd0;
            m_reg <= 32'd0;
            y_reg <= 32'd0;
        end
        else begin
            b_reg <= b;
            x_reg <= x;
            m_reg <= m;
            if (yn_count == tap_number)
                y_reg <= 0;
            else
                y_reg <= y;
        end
    end
endmodule
