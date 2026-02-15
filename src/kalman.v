`default_nettype none

module kalman #(
    parameter RATE_SHIFT = 6, // Approx division by 64 for dt * scale
    parameter K_SHIFT = 6     // Approx Gain 0.015 (1/64)
)(
    input  wire             clk,
    input  wire             rst_n,
    input  wire             en, // Update enable (e.g. valid signal)
    input  wire signed [15:0] rate,    // Gyro Rate
    input  wire signed [15:0] angle_m, // Measured Angle (Accel)
    output reg  signed [15:0] angle_out
);

    // Intermediate calculations
    // Use wires to avoid latches

    // Prediction: Angle = Angle + Rate * dt
    wire signed [15:0] rate_term = rate >>> RATE_SHIFT;
    wire signed [15:0] pred_angle = angle_out + rate_term;

    // Innovation: Error = Meas - Pred
    wire signed [15:0] innov = angle_m - pred_angle;

    // Update: Angle = Pred + K * Innovation
    wire signed [15:0] update_term = innov >>> K_SHIFT;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            angle_out <= 0;
        end else if (en) begin
            angle_out <= pred_angle + update_term;
        end
    end

endmodule
