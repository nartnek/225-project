package rubikscube;


final class Constants {

    private Constants() {
    }

    static final int N_SLICE = (12 * 11 * 10 * 9) / (4 * 3 * 2 * 1);

    //3^7
    static final int N_TWIST = (int) Math.round(Math.pow(3, 7));

    static final int N_TWIST_SYM = 324;

    //2^11
    static final int N_FLIP = 1 << 11;

    static final int N_FLIP_SYM = 336;

    //8!
    static final int N_PERM = 40320;

    static final int N_PERM_SYM = 2768;

    //4!
    static final int N_MPERM = 24;

    static final int N_MOVES_PHASE1 = 18;
    static final int N_MOVES_PHASE2 = 10;

    static final int N_COMB = PhaseSolver.USE_COMBP_PRUN ? 140 : 70;

    static final int P2_PARITY_MOVE = PhaseSolver.USE_COMBP_PRUN ? buildParityMask() : 0;

    private static int buildParityMask() {
        int result = 0;
        result |= (1 << 0);
        result |= (1 << 2);
        result |= (1 << 5);
        result |= (1 << 7);
        return result;
    }

    static final int PRUN_UNIT;

    static final int PRUN_ZERO_MASK;

    static {
        int u = 0;
        for (int i = 0; i < 8; i++) {
            u |= (1 << (i * 4));
        }
        PRUN_UNIT = u;

        int z = 0;
        for (int i = 0; i < 8; i++) {
            z |= (1 << (i * 4 + 3));
        }
        PRUN_ZERO_MASK = z;
    }


    static final int SKIP_MAGIC_PHASE2 = Integer.parseInt("42", 16);

    static final int SKIP_MAGIC_PHASE1 = (1 << 17) | (1 << 13) | (1 << 9) | (1 << 5) | (1 << 1);


    static final int PRUNFLAG_TWIST_FLIP = Integer.parseInt("19603", 16);


    static final int PRUNFLAG_SLICE_TWIST = Integer.parseInt("69603", 16);
    static final int PRUNFLAG_SLICE_FLIP = Integer.parseInt("69603", 16);

    static final int PRUNFLAG_MC_PERM = Integer.parseInt("8EA34", 16);

    static final int PRUNFLAG_EPERM_COMBP = Integer.parseInt("7D824", 16);
}
