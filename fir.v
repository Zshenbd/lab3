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

// FSM ap_ctrl----------------------------------------------------------------------------- 
    reg [2:0]  ap_ctrl; //bit 0: ap_start, bit 1: ap_done, bit 2: ap_idle
    reg [2:0]  next_ap_ctrl;
    reg [1:0]  ap_state;
    reg [1:0]  next_ap_state;
    `define AP_RUN 2'b00
    `define AP_IDLE 2'b01
    `define AP_DONE 2'b10
    
    always @* begin
        case (ap_state)
            `AP_IDLE:
            begin
                if (awaddr == 12'd0 && wdata[0] == 1 && yn_total_count != data_length) begin
                    next_ap_state <= `AP_RUN;
                    next_ap_ctrl <= 3'b101;
                end else begin
                    next_ap_state <= `AP_IDLE;
                    next_ap_ctrl <= 3'b100;
                end  
            end
            `AP_RUN:
            begin
                if (sm_tvalid && sm_tlast) begin
                    next_ap_state <= `AP_DONE;
                    next_ap_ctrl <= 3'b010;
                end else begin
                    next_ap_state <= `AP_RUN;
                    next_ap_ctrl <= 3'b000;
                end
            end
            `AP_DONE:
            begin
                if (araddr == 12'd0 && arvalid && rvalid) begin
                    next_ap_state <= `AP_IDLE;
                    next_ap_ctrl <= 3'b100;
                end else begin
                    next_ap_state <= `AP_DONE;
                    next_ap_ctrl <= 3'b010;
                end
            end
            default:begin
                next_ap_state <= `AP_IDLE;
                next_ap_ctrl <= 3'b100;
            end
        endcase
    end
    
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            ap_state <= `AP_IDLE;
            ap_ctrl <= 3'b100;
        end else begin
            ap_state <= next_ap_state;
            ap_ctrl <= next_ap_ctrl;
        end
    end

// AXI-Lite----------------------------------------------------------------------------- 
    reg fir_awready;
    reg fir_wready;
    reg fir_arready;
    reg fir_rvalid;
    
    reg [31:0] data_length;
    reg [31:0] tap_awdata;
   
    assign awready = fir_awready;
    assign wready = fir_wready;
    assign arready = fir_arready;
    assign rvalid = fir_rvalid;
    
    assign tap_EN = (axis_rst_n)? 1 : 0;
    assign tap_WE = ((wvalid == 1) && (awaddr[7:0] != 0))? 4'b1111 : 4'b0000;
    assign tap_A  = (awvalid == 1 && (awaddr[5] == 1 || awaddr[6] == 1))? (awaddr[6:0]-6'h20) : tap_ar[5:0];
    assign tap_Di = (awvalid == 1 && (awaddr[5] == 1 || awaddr[6] == 1))? wdata : 32'h00000000;
    assign rdata = (araddr[7:0] == 8'd0)? ap_ctrl : tap_Do;
        
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            fir_awready <= 0;
            fir_wready <= 0;
            fir_arready <= 0;
            fir_rvalid <= 0;
        end else begin
            fir_awready <= (awvalid && wvalid)? 1 : 0;
            fir_wready <= (awvalid && wvalid)? 1 : 0;
            fir_arready <= (arvalid && rready)? 1 : 0;
            fir_rvalid <= (arvalid && rready)? 1 : 0;
        end
    end
    
    always @* begin
        if (fir_awready && fir_wready) begin
            if (awaddr[7:0] == 8'h10) data_length <= wdata;
            else data_length <= data_length;
        end else data_length <= data_length;
    end
    
// AXI_stream_X[n]-----------------------------------------------------------------------------      
    
    assign ss_tready = (ss_tvalid && !ap_ctrl[2] && xn_count[3:0] == 4'd0)? 1 : 0;
    assign data_EN = (axis_rst_n)? 1 : 0; 
    assign data_WE = (ss_tready || (ap_ctrl[2] && i_data_a != 6'd44 ))? 4'b1111 : 4'b0000;
    assign data_A = (ap_ctrl[2])? i_data_a : fir_data_a;
    assign data_Di = (ap_ctrl[2])? 0 : ss_tdata;
    
     //initialize
    reg  [5:0] i_data_a;
    
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if(!axis_rst_n && !data_EN) i_data_a <= -6'd04;
        else begin
            if (i_data_a == 6'd44) i_data_a <= 6'd44;
            else i_data_a <= i_data_a + 6'd04;
        end
    end

    reg ss_state;
    reg ss_next_state;
    
    always @* begin
        case(ss_state)
        1'b1: //x[n] is writing into the data_RAM 
        begin
            if (ss_tvalid && ss_tlast) ss_next_state = 0;
            else ss_next_state = 1;
        end
        1'b0: //x[n] finish writing into the data_RAM
        begin
            if (ss_tvalid) ss_next_state = 1;
            else ss_next_state = 0;
        end
        default: ss_next_state = (ss_tvalid)? 1 : 0;
        endcase
    end
    
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n)
            ss_state <= 0;
        else
            ss_state <= ss_next_state;
    end
    
// AXI_stream_Y[n]-----------------------------------------------------------------------------          
    reg sm_state;
    reg sm_next_state;
    reg [3:0] yn_count;
    reg [31:0] yn_total_count;
    
    assign sm_tvalid = (yn_count == 4'd1)? 1 : 0;    
    assign sm_tdata  = (sm_tvalid)? y_reg : 0;             
    assign sm_tlast  = (sm_tvalid && yn_total_count == data_length - 1)? 1 : 0; 
    
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n || ap_ctrl[2])
            yn_count = 4'd0;
        else begin
            if (yn_count == 4'd0) yn_count = Tape_Num + 4'd3;
            else if (yn_count == 4'd1) yn_count = Tape_Num;
            else yn_count = yn_count - 1;
        end
    end
    
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            yn_total_count <= 32'd0;
        end 
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
    
    always @* begin
        case(sm_state)
        1'b1: //y[n] is read out 
        begin
            if (sm_tlast) sm_next_state = 0;
            else sm_next_state = 1;
        end
        1'b0: //y[n] finish calculating and read
        begin
            if (sm_tready) sm_next_state = 1;
            else sm_next_state = 0;
        end
        default: sm_next_state = (sm_tvalid)? 1 : 0;
        endcase
    end
    
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n)
            sm_state <= 0;
        else
            sm_state <= sm_next_state;
    end
    
    
//tap RAM Address Generator----------------------------------------------------------------------------- 
    wire [5:0] tap_ar;    // address which will send into tap_RAM
    reg  [5:0] fir_tap_ar;
    
    assign tap_ar = (ap_ctrl[2])? (araddr[6:0]-6'h20) : fir_tap_ar; 

    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n || ap_ctrl[2])
            fir_tap_ar = 6'd00;
        else begin 
            if (fir_tap_ar == (Tape_Num - 1)*4) fir_tap_ar = 6'd00;
            else  fir_tap_ar = fir_tap_ar + 3'd4;
        end
    end
    
//data RAM Address Generator-----------------------------------------------------------------------------

    reg [3:0] xn_count;
    reg [5:0] fir_data_aw;
    reg [5:0] fir_data_ar;
    wire [5:0] fir_data_a;
    
    assign fir_data_a = (ss_tready) ? fir_data_aw : fir_data_ar;
    
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n || ap_ctrl[2])
            xn_count = 4'd0;
        else begin
            if (xn_count == 4'd0) xn_count = Tape_Num - 1;
            else xn_count = xn_count - 1;
        end
    end
    
    //address write to data_ram
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n || ap_ctrl[2]) 
            fir_data_aw = 6'd04;
        else begin
            if (xn_count == 4'd0) begin
                if (fir_data_aw == (Tape_Num - 1)*4) fir_data_aw = 6'd00;
                else  fir_data_aw = fir_data_aw + 6'd04;
            end else fir_data_aw = fir_data_aw;
        end
    end
    
    //address read data_ram
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n || ap_ctrl[2]) 
            fir_data_ar = 6'd04;
        else begin 
            if (xn_count != 4'd0) begin
                if (fir_data_ar == 6'd00) fir_data_ar = (Tape_Num - 1)*4;
                else fir_data_ar = fir_data_ar - 6'd04;
            end else fir_data_ar = fir_data_ar;
        end
    end
  
// FIR operation-----------------------------------------------------------------------------

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
            
    // Operation
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n || ap_ctrl[2]) begin
            b_reg <= 32'd0;
            x_reg <= 32'd0;
            m_reg <= 32'd0;
            y_reg <= 32'd0;
        end
        else begin
            b_reg <= b;
            x_reg <= x;
            m_reg <= m;
            if (yn_count == Tape_Num)
                y_reg <= 0;
            else
                y_reg <= y;
        end
    end

endmodule
