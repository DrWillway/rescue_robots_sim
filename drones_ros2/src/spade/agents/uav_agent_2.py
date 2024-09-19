import spade
import drone_explore as agent_drone_explore

async def main(args=None):
    
    drone = agent_drone_explore.FSMAgent("rescure_drone@jabbers.one", "rescure_drone")
    drone.id = 2
    drone.receiver_jid1 = "testesmagia@jabbers.one"
    drone.receiver_jid2 = "rescue_drone2@jabbers.one"
    
    # Start both functionalities concurrently
    future = drone.start()
    await future

    await spade.wait_until_finished(drone)

    await drone.stop()

    print("Agents finished")

if __name__ == "__main__":
    spade.run(main())