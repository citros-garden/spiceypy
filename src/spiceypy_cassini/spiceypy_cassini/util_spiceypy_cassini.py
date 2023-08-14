import spiceypy as spice

def run(ros_node, start_t, finish_t, num_steps):

    ros_node.get_logger().info(f"Starting simulation...")

    spice.furnsh("/workspaces/spiceypy/src/spiceypy_cassini/resource/cassMetaK.txt")

    step = num_steps
    # we are going to get positions between these two dates
    utc = [start_t, finish_t]

    # get et values one and two, we could vectorize str2et
    etOne = spice.str2et(utc[0])
    etTwo = spice.str2et(utc[1])
    # print("ET One: {}, ET Two: {}".format(etOne, etTwo))

    # get times
    times = [x*(etTwo-etOne)/step + etOne for x in range(step)]

    #Run spkpos as a vectorized function
    positions, lightTimes = spice.spkpos('Cassini', times, 'J2000', 'NONE', 'SATURN BARYCENTER')

    # Clean up the kernels
    spice.kclear()

    ros_node.get_logger().info(f"Simulation finished successfully!")

    return (positions)