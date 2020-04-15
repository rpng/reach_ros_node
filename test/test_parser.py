from reach_ros_node import parser


def test_parse_llh_sentence(test_data_dir):
    # Load test data.
    llh_sentence = ''
    with open(str(test_data_dir.join('llh.txt')), 'r') as f:
        llh_sentence = f.readlines()[1]

    # Call.
    parsed = parser.parse_llh_sentence(llh_sentence)

    assert type(parsed) == dict

    # Check that the covariance matrix matches the test data.
    stdev = [0.0117, 0.0043, 0.0130,
            0.0043, 0.0072, 0.0029,
            0.0130, 0.0029, 0.0242]
    cov = map(lambda sigma: sigma ** 2, stdev)
    assert parsed['position_covariance'] == cov
