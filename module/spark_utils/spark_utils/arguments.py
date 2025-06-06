import argparse

def parse_args():

    # Create the parser
    parser = argparse.ArgumentParser(description="Spark Args")

    # Add arguments
    # parser.add_argument("--name", type=str, help="Your name", required=True)
    # parser.add_argument("--age", type=int, help="Your age", required=True)
    # parser.add_argument("--verbose", action="store_true", help="Enable verbose output")
    parser.add_argument("--safe_algo", type=str, default='ssa', help="ssa/sss/cbf/etc.", required=False)

    # Parse arguments
    args = parser.parse_args()

    return args