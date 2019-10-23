from reach_ros_node import parser


def test_parse_llh_sentence(test_data_dir):
    # Load test data.
    llh_sentence = ''
    with open(str(test_data_dir.join('llh.txt')), 'r') as f:
        llh_sentence = f.readlines()[1]

    # Run the test; do some basic sanity checks.
    parsed = parser.parse_llh_sentence(llh_sentence)
    assert type(parsed) == dict
    assert len(parsed.keys()) == len(parser.parse_maps["LLH"])
