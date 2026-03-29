package modules.Application;
@namespace(quisp::modules);

simple QSDCApplication
{
  parameters:
    @display("i=block/app");
    int number_of_bellpair;
    int bob_addr = default(1);

    int min_pairs_to_start = default(8);
    int sample_target = default(32);
    bool expect_anti = default(false);

    int bell_sample_target = default(16);

    int sample_block_size = default(8);
    int bell_block_size = default(8);

    double start_delay @unit(s) = default(0s);
    double poll_interval @unit(s) = default(0s);
    double sample_interval @unit(s) = default(0s);
    int burn_count;

    double phase1_max_error_rate = default(0.1);
	double phase2_max_error_rate = default(0.1);

    string payload = default("hello world");

    // Eve settings
    bool eve_enabled = default(false);
    double eve_intercept_probability = default(1.0);

  gates:
    input fromRouter @loose;
    output toRouter @loose;

    input fromQuantum @loose;
    output toQuantum @loose;
}