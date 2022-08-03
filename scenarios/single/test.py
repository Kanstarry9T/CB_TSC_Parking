import argparse
def test():
    args = ['-n', 'net.xml', '-o', 'trip.xml']

    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--net-file", dest="netfile",
                           help="define the net file (mandatory)")

    parser.add_argument("-o", "--output-trip-file", dest="tripfile",
                           default="trips.trips.xml", help="define the output trip filename")
    options = parser.parse_args(args=args)
    print(options.netfile)
    print(options.tripfile)
test()