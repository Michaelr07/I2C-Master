
interface vid_sideband_if
    #(
        parameter int XW = 10,
        parameter int YW = 10
    )
    (
    );
    
    logic [XW-1:0] x;
    logic [YW-1:0] y;
    logic de;
    logic sof;
    logic eol;

    modport source (
       // input clk, rst_n,
        output de, sof, eol, x, y
    );
    
    modport sink (
        input  de, sof, eol, x, y
    );
    
    endinterface
    


