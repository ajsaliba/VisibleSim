/**
 * @file simulator.h
 * @brief Declares BaseSimulator::Simulator.
 *
 * The Simulator is the entry point for a VisibleSim run. It:
 * - Parses command-line arguments and the XML configuration file.
 * - Instantiates the simulation World (model-specific).
 * - Instantiates and configures the event Scheduler.
 * - Instantiates all modules (blocks) and optional objects (targets, obstacles, visuals).
 * - Starts the main loop (GUI or terminal mode).
 */

#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#define TIXML_USE_STL    1

#include "../deps/TinyXML/tinyxml.h"

#include "../utils/tDefs.h"
#include "../events/scheduler.h"
#include "world.h"
#include "../utils/commandLine.h"
#include "blockCode.h"
#include "../replay/replayExporter.h"

using namespace std;

namespace BaseSimulator {

    class Simulator;

    extern Simulator *simulator;

    /**
     * @class Simulator
     * @brief Orchestrates configuration parsing and bootstraps a simulation run.
     *
     * Lifecycle (typical):
     * - Construct Simulator(argc, argv, blockCodeBuilder)
     * - parseConfiguration(argc, argv)  (loads world, scheduler, blocks, obstacles, targets, customizations)
     * - startSimulation()               (links blocks, starts scheduler, enters main loop)
     * - deleteSimulator()               (destroys world/scheduler/config document)
     *
     * Ownership:
     * - Owns the configuration XML document (xmlDoc).
     * - Owns the instantiated World and Scheduler.
     * - Maintains a global singleton pointer (BaseSimulator::simulator) for convenience.
     */
    class Simulator {
    public:
        enum IDScheme {
            ORDERED = 0, MANUAL, RANDOM
        };

        //! True when running in regression-testing mode.
        static bool regrTesting;

        //! When enabled, exports the final configuration before termination.
        inline static bool exportFinalConfiguration;

        //! Name/path of the configuration file currently loaded.
        inline static string configFileName;

        static Simulator *getSimulator() {
            assert(simulator != NULL);
            return (simulator);
        }

        /** @brief Deletes the singleton simulator instance (if any). */
        static void deleteSimulator();

        inline CommandLine &getCmdLine() { return cmdLine; }

        /** @brief Debug helper. Subclasses may override. */
        virtual void printInfo() { cout << "Simulator" << endl; }

        /**
         * @brief Parses the XML configuration and configures the simulation.
         *
         * This creates/configures the World and Scheduler, then instantiates all blocks and
         * optional elements (obstacles, targets, customizations).
         */
        void parseConfiguration(int argc, char *argv[]);

        /**
         * @brief Starts the simulation loop.
         *
         * Links blocks, starts the scheduler if auto-start is enabled, and enters the
         * main loop (GLUT when GUI is enabled).
         */
        void startSimulation();

        /** @brief Returns a random unsigned integer for simulation use. */
        ruint getRandomUint();

        /**
         * @brief Returns the effective simulation seed.
         *
         * Note: this is the authoritative seed for the run. Command-line may not specify it.
         */
        inline int getSimulationSeed() { return seed; }

    protected:
        //! Simulation seed used for randomized operations.
        int seed;

        //! Random generator used for simulation randomness (not for ID distribution).
        uintRNG generator;

        //! Singleton instance pointer.
        static Simulator *simulator;

        //! Scheduler instance used to execute events.
        Scheduler *scheduler;

        //! Simulation world (model-specific).
        World *world;

        //! TinyXML document representing the configuration file.
        TiXmlDocument *xmlDoc;

        //! Cached pointer to the <blockList> node, used during parsing.
        TiXmlNode *xmlBlockListNode;

        //! Factory for per-module program (BlockCode) instances.
        BlockCodeBuilder bcb;

        //! Parsed command-line arguments.
        CommandLine cmdLine;

        //! Maximum simulation date (if bounded). This may also be set by deprecated XML attributes.
        int schedulerMaxDate = 0;

        /**
         * Pool of IDs to assign to blocks.
         *
         * - ORDERED: IDs are assigned sequentially as blocks are encountered.
         * - MANUAL: IDs are read from each <block id="...">.
         * - RANDOM: IDs are generated and shuffled.
         */
        vector<bID> IDPool;

        //! ID distribution scheme currently in use.
        IDScheme ids = ORDERED;

        /**
         * @brief Parses a <vs> root node (if present) and then the <world> node.
         * @return Pointer to the parsed <world> node, or nullptr on failure.
         */
        TiXmlNode *parseVS(TiXmlNode *parent, int argc, char *argv[]);

    /**
     * @brief Parses the <world> node and instantiates the model-specific World.
     * @return Pointer to the parsed <world> node, or nullptr if missing.
     */
        TiXmlNode *parseWorld(TiXmlNode *parent, int argc, char *argv[]);

    /** @brief Parses optional <visuals> settings (window, background, render toggles). */
        bool parseVisuals(TiXmlNode *parent);

        /*!
         *  @brief Examines the configuration file's blockList attribute and determine the ID distribution scheme to be used
         *  @return the enum IDScheme value corresponding to the scheme to be used. ORDERED, if none specified in the configuration.
         *  @attention The xmlBlockListNode attribute has to be initialized before calling this function.
         */
        IDScheme determineIDScheme();

        /*!
         * @brief Counts the number of modules defined in the configuration file. From both blocksLine and block elements.
         * @return The number of modules defined in configuration file.
         */
        bID countNumberOfModules();

        /*!
         *  @brief Initialize the pool of id according to the ID assignment model specified in configuration file
         *  If it does not exist, initialize IDPool with with contiguous integers up to N {1,2,3,...,N}
         *  @attention The xmlBlockListNode attribute has to be initialized before calling this function.
         */
        void initializeIDPool();

        /*!
         *  @brief Initializes IDPool with n ID distanced by step and shuffled
         *  @param n the number of IDs to generate
         *  @param idSeed the random seed used to configure the random number generator. If idSeed = -1, a random seed is used instead.
         *  @param step the distance between two consecutive numbers. e.g. If n = 4 and step = 2 then IDPool = {1, 3, 5, 7}
         *  @attention The user has to ensure that the generation won't cause any overflow.
         *  If (1 + (n + 1) * step) > BID_MAX (i.e. 1.8446744e+019 with UINT64_T as bID), undefined behavior will happen.
         */
        void generateRandomIDs(const int n, const int idSeed, const int step);

        /*!
         *  @brief Parses the configuration file's blockList attribute for the random seed attribute
         *  @return The integer seed specified in the configuration file, or -1 if unspecified
         *  @throw ParsingException in case the seed is not a valid integer number
         */
        int parseRandomIdSeed();

        /*!
         *  @brief Parses the configuration file's blockList attribute for the random step attribute
         *  @return The integer step specified in the configuration file, or 1 if unspecified
         *  @throw ParsingException in case the step is not a valid integer number
         */
        bID parseRandomStep();

        /**
         *  @brief Parses the configuration file's attributes related to the behavioral customization of VisibleSim (e.g., adjusting the rotation speed of catoms)
         *  @throw ParsingException if any of the customization variable is ill-formatted
         */
        void parseCustomizations(TiXmlNode *parent);

        /*! @fn loadScheduler(int maximumDate)
     *  @brief Instantiates a scheduler instance for the simulation based on the type of CodeBlock
     *
     *  Only c++ is supported
     *
     *  @param maximumDate : maximum simulation date none by default
     *
     */
        void loadScheduler(int maximumDate = 0);

        /*! @fn parseCameraAndSpotlight();
         *  @brief Parses the configuration file for Camera and Spotlight information
         *
         *  Calls the loadWorld virtual function to instantiate the right subclass of World with the parsed data.
         *
         *
         */
        bool parseCameraAndSpotlight(TiXmlNode *parent);

        /*! @fn parseBlockList()
         *  @brief Parses the configuration for block information common to all blocks
         *
         *  Calls the loadBlock virtual function once for every node to instantiate.
         *
         */
        void parseBlockList();

        /*!
          *  @brief Parses the configuration for obstacles information
          *
          */
        void parseObstacles(TiXmlNode *parent);

        /** @brief Parses optional targets (<targetList>) and initializes BlockCode::target. */
        void parseTarget(TiXmlNode *parent);

        /*! @fn virtual void loadWorld(int lx, int ly, int lz, int argc, char *argv[])
         *  @brief Calls the createWorld function from the target world subclass to instantiate it
         *
         *  @param gridSize the size of the simulation grid
         *  @param gridScale the real size of a block
         *  @param argc The number of command line arguments
         *  @param argv The command line arguments
         *
         */
        virtual void loadWorld(const Cell3DPosition &gridSize, const Vector3D &gridScale,
                               int argc, char *argv[]) = 0;

        /**
         *  @brief Parses the config file for any required additional block attribute, and add it to the world
         *  @param blockElt The current block XML element for parsing additional attributes
         *  @param blockId id of the block to add
         *  @param buildingBlockCodeBuildingFunction function pointer to the user blockCode
         *  @param pos Position of the block to add
         */
        virtual void loadBlock(TiXmlElement *blockElt, bID blockId, BlockCodeBuilder bcb,
                               const Cell3DPosition &pos, const Color &color, uint8_t orient) = 0;

        /**
         * @brief Constructs a Simulator and loads the configuration XML document.
         *
         * The World and blocks are not created until parseConfiguration().
         */
        Simulator(int argc, char *argv[], BlockCodeBuilder bcb);

        virtual ~Simulator();

    public:
        /**
         *  @brief Getter for the configuration file TiXmlDocument
         *  @return a pointer to the configuration file TinyXml doc
         */
        inline TiXmlDocument *getConfigDocument() { return xmlDoc; }

        inline BlockCodeBuilder getBlockCodeBuilder() { return bcb; }

        static Cell3DPosition extractCell3DPositionFromString(string str);

        static pair<int, int> extract2DpointFromString(string str);

        static Vector3D extractVector3DFromString(string str);

        static Color extractColorFromString(string str);

        static bool extractBoolFromString(string str);

        static int extractIntFromString(string str);
    };

    inline void deleteSimulator() {
        Simulator::deleteSimulator();
    }

} // BaseSimulator namespace

#endif /* SIMULATOR_H_ */
