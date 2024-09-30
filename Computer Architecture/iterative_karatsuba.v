/* 32-bit simple karatsuba multiplier */

/*32-bit Karatsuba multipliction using a single 16-bit module*/

module iterative_karatsuba_32_16(clk, rst, enable, A, B, C , g1);
    input clk;
    input rst;
    input [31:0] A;
    input [31:0] B;
    output [63:0] C;
    
    input enable;
    
    
    wire [1:0] sel_x;
    wire [1:0] sel_y;
    
    wire [1:0] sel_z;
    wire [1:0] sel_T;
    
    
    wire done;
    wire en_z;
    wire en_T;
    
    
    wire [32:0] h1;
    wire [32:0] h2;
    output [63:0] g1;
    wire [63:0] g2;
    
    assign C = g2;
    reg_with_enable #(.N(63)) Z(.clk(clk), .rst(rst), .en(en_z), .X(g1), .O(g2) );  // Fill in the proper size of the register
    reg_with_enable #(.N(32)) T(.clk(clk), .rst(rst), .en(en_T), .X(h1), .O(h2) );  // Fill in the proper size of the register
    
    iterative_karatsuba_datapath dp(.clk(clk), .rst(rst), .X(A), .Y(B), .Z(g2), .T(h2), .sel_x(sel_x), .sel_y(sel_y), .sel_z(sel_z), .sel_T(sel_T), .en_z(en_z), .en_T(en_T), .done(done), .W1(g1), .W2(h1));
    iterative_karatsuba_control control(.clk(clk),.rst(rst), .enable(enable), .sel_x(sel_x), .sel_y(sel_y), .sel_z(sel_z), .sel_T(sel_T), .en_z(en_z), .en_T(en_T), .done(done));
    
endmodule

module iterative_karatsuba_datapath(clk, rst, X, Y, T, Z, sel_x, sel_y, en_z, sel_z, en_T, sel_T, done, W1, W2);
    input clk;
    input rst;
    input [31:0] X;    // input X
    input [31:0] Y;    // Input Y
    input [32:0] T;    // input which sums X_h*Y_h and X_l*Y_l (its also a feedback through the register)
    input [63:0] Z;    // input which calculates the final outcome (its also a feedback through the register)
    output [63:0] W1;  // Signals going to the registers as input
    output [32:0] W2;  // signals going to the registers as input
    
    
    input [1:0] sel_x;  // control signal 
    input [1:0] sel_y;  // control signal 
    
    input en_z;         // control signal 
    input [1:0] sel_z;  // control signal 
    input en_T;         // control signal 
    input [1:0] sel_T;  // control signal 
    
    input done;         // Final done signal
   
    reg[15:0] mul_x,mul_y;
    wire[31:0] mul_out;
    reg[15:0] X_lh,Y_hl;
    reg[63:0] XY; //to store x_h*y_h and x_l*y_l
    reg[31:0] XY_hl;//to store (X_l-X_h)*(Y_h-Y_l)
    wire[32:0] add_out1;
    reg[31:0] x1,x2;
    reg[32:0] x3,x4;
    wire [32:0]add_out2,sub_out1;
    reg[32:0] x5,x6;
    reg [32:0] result;
    reg[63:0] x9,x13;
    reg[15:0] y1 , y2 , j1 , j2 ;
    wire[15:0] l1 ,l2 ; 
    reg [32:0] store_addout1 ;

    wire a,b;
    reg c; //sign of the subtractions X_l-X_h and Y_h-Y_l
    
    // multiplier defined 
    mult_16 mul(.X(mul_x),.Y(mul_y),.Z(mul_out));

    subtract_Nbit #(16) sub1(.a(y1),.b(j1),.cin(1'b0),.S(l1),.ov(),.cout_sub(a));
    subtract_Nbit #(16) sub2(.a(y2),.b(j2),.cin(1'b0),.S(l2),.ov(),.cout_sub(b));
    
    // To add XhYh_XlYl 
    adder_Nbit #(33) add1(.a({1'b0,x1}),.b({1'b0,x2}),.cin(1'b0),.S(add_out1),.cout());
    adder_Nbit #(33) add2(.a(x3),.b(x4),.cin(1'b0),.S(add_out2[32:0]),.cout()); 
    
    subtract_Nbit #(33) sub3(.a(x5),.b(x6),.cin(1'b0),.S(sub_out1),.ov(),.cout_sub());
    
    adder_Nbit #(64) add4(.a(x9),.b(x13),.cin(1'b0),.S(W1),.cout());

    always @(*) begin
        case(sel_x)
            2'b00:begin
                mul_x = X[31:16];
                mul_y = Y[31:16];
                XY[63:32] = mul_out;
            end
            2'b01:begin
                y1 = X[15:0] ;
                j1 = X[31:16] ;
                y2 = Y[31:16] ;
                j2 = Y[15:0] ;
                X_lh = l1 ;
                Y_hl = l2 ; 
                c = a^b ;
                mul_x = X[15:0];
                mul_y = Y[15:0];
                XY[31:0] = mul_out;
            end
            2'b10:begin
                mul_x = X_lh;
                mul_y = Y_hl;
                XY_hl = mul_out;
            end
        endcase
    end
    
    always @(*) begin
        case(sel_y)
            
            2'b00: begin
                x1 = XY[63:32];
                x2 = XY[31:0];
                store_addout1 = add_out1 ;
            end

            2'b01:begin
                x3 = (c==1'b0)?add_out1  : 33'b0 ; 
                x4=(c==1'b0)?{1'b0,XY_hl}:33'b0;

                x5 = (c==1'b1)?add_out1:33'b0;
                x6 = (c==1'b1)?{1'b0,XY_hl}:33'b0;
                
                result = (c==1'b1)?sub_out1:add_out2;
            end
        endcase
        case (sel_z)
            2'b10:begin
                x9 =  XY;
                x13 = {15'b0,result,16'b0} ; 
            end
            default : begin 
                x9 = 64'b0 ;
                x13 = 64'b0 ;
            end
        endcase
    end
endmodule


module iterative_karatsuba_control(clk,rst, enable, sel_x, sel_y, sel_z, sel_T, en_z, en_T, done);
    input clk;
    input rst;
    input enable;
    
    output reg [1:0] sel_x;
    output reg [1:0] sel_y;
    
    output reg [1:0] sel_z;
    output reg [1:0] sel_T;    
    
    output reg en_z;
    output reg en_T;
    
    
    output reg done;
    
    reg [5:0] state, nxt_state;
    parameter S0 = 6'b000001;  // initial state
    parameter S1 = 6'b000010;
    parameter S2 = 6'b000011;
    parameter S3 = 6'b000100;
    parameter S4 = 6'b000101;
    parameter S5 = 6'b000110;
   // <define the rest of the states here>

    always @(posedge clk) begin
        if (rst) begin
            state <= S0;
        end
        else if (enable) begin
            state <= nxt_state;
        end
    end
    

    always@(*) begin
        case(state) 
            S0: 
                begin
					// Write your output and next state equations here
                    sel_x = 2'b00; //computing X_h*Y_h
                    nxt_state <= S1;
                end
			// Define the rest of the states
            S1:
                begin
                sel_x = 2'b01; //computing Y_h*Y_l
                nxt_state <= S2;
                end
            S2:
                begin
                sel_x = 2'b10 ; //computing (X_l-X_h)*(Y_h-Y_l)
                sel_y =2'b00 ; //computing (x_h*x_l)+(y_h*y_l)
                nxt_state <= S3;
                end
            S3 :
                begin
                sel_y = 2'b01; //computing (x_h*x_l)+(y_h*y_l)+(x_l-x_h)*(y_h-y_l)
                nxt_state <= S4 ; 
                end
            S4:
                begin
                    sel_z = 2'b10;
                    en_z <= 1'b1;
                    done <= 1'b1 ; 
                end
            default: 
                begin
				// Don't forget the default
                sel_x <= 2'b00;
                en_z <= 1'b0;
                en_T <= 1'b0;
                done <= 1'b0;
                nxt_state <= S0;
                end            
        endcase
        
    end

endmodule


module reg_with_enable #(parameter N = 32) (clk, rst, en, X, O );
    input [N:0] X;
    input clk;
    input rst;
    input en;
    output [N:0] O;
    
    reg [N:0] R;
    
    always@(posedge clk) begin
        if (rst) begin
            R <= {N{1'b0}};
        end
        if (en) begin
            R <= X;
        end
    end
    assign O = R;
endmodule







// /-------------------Supporting Modules--------------------/
// /------------- Iterative Karatsuba: 32-bit Karatsuba using a single 16-bit Module/

module mult_16(X, Y, Z);
input [15:0] X;
input [15:0] Y;
output [31:0] Z;

assign Z = X*Y;

endmodule


module mult_17(X, Y, Z);
input [16:0] X;
input [16:0] Y;
output [33:0] Z;

assign Z = X*Y;

endmodule

module full_adder(a, b, cin, S, cout);
input a;
input b;
input cin;
output S;
output cout;

assign S = a ^ b ^ cin;
assign cout = (a&b) ^ (b&cin) ^ (a&cin);

endmodule


module check_subtract (A, B, C);
 input [7:0] A;
 input [7:0] B;
 output [8:0] C;
 
 assign C = A - B; 
endmodule



/* N-bit RCA adder (Unsigned) */
module adder_Nbit #(parameter N = 32) (a, b, cin, S, cout);
input [N-1:0] a;
input [N-1:0] b;
input cin;
output [N-1:0] S;
output cout;

wire [N:0] cr;  

assign cr[0] = cin;


generate
    genvar i;
    for (i = 0; i < N; i = i + 1) begin
        full_adder addi (.a(a[i]), .b(b[i]), .cin(cr[i]), .S(S[i]), .cout(cr[i+1]));
    end
endgenerate    


assign cout = cr[N];

endmodule


module Not_Nbit #(parameter N = 32) (a,c);
input [N-1:0] a;
output [N-1:0] c;

generate
genvar i;
for (i = 0; i < N; i = i+1) begin
    assign c[i] = ~a[i];
end
endgenerate 

endmodule


/* 2's Complement (N-bit) */
module Complement2_Nbit #(parameter N = 32) (a, c, cout_comp);

input [N-1:0] a;
output [N-1:0] c;
output cout_comp;

wire [N-1:0] b;
wire ccomp;

Not_Nbit #(.N(N)) compl(.a(a),.c(b));
adder_Nbit #(.N(N)) addc(.a(b), .b({ {N-1{1'b0}} ,1'b1 }), .cin(1'b0), .S(c), .cout(ccomp));

assign cout_comp = ccomp;

endmodule


/* N-bit Subtract (Unsigned) */
module subtract_Nbit #(parameter N = 32) (a, b, cin, S, ov, cout_sub);

input [N-1:0] a;
input [N-1:0] b;
input cin;
output [N-1:0] S , S_temp , S_comp;
output ov;
output cout_sub;

wire [N-1:0] minusb;
wire cout;
wire ccomp;

Complement2_Nbit #(.N(N)) compl(.a(b),.c(minusb), .cout_comp(ccomp));
adder_Nbit #(.N(N)) addc(.a(a), .b(minusb), .cin(1'b0), .S(S_temp), .cout(cout));

Complement2_Nbit #(.N(N)) comp2(.a(S_temp) , .c(S_comp) , .cout_comp()) ; 

assign ov = (~(a[N-1] ^ minusb[N-1])) & (a[N-1] ^ S_temp[N-1]);

assign cout_sub = cout | ccomp;

assign S = (cout_sub == 1'b0) ? S_comp : S_temp;

endmodule

/* n-bit Left-shift */

module Left_barrel_Nbit #(parameter N = 32)(a, n, c);

input [N-1:0] a;
input [$clog2(N)-1:0] n;
output [N-1:0] c;


generate
genvar i;
for (i = 0; i < $clog2(N); i = i + 1 ) begin: stage
    localparam integer t = 2**i;
    wire [N-1:0] si;
    if (i == 0) 
    begin 
        assign si = n[i]? {a[N-t:0], {t{1'b0}}} : a;
    end    
    else begin 
        assign si = n[i]? {stage[i-1].si[N-t:0], {t{1'b0}}} : stage[i-1].si;
    end
end
endgenerate

assign c = stage[$clog2(N)-1].si;

endmodule
