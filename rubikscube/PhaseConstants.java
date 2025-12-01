package rubikscube;

public final class PhaseConstants {

    private PhaseConstants() {
    }

    public static final int NUM_NODES = 21;

    public static final int NUM_URF = 6;

    public static final int USE_SEPARATOR = 0x1;
    public static final int INVERSE_SOL  = 0x2;
    public static final int APPEND_LEN   = 0x4;
    public static final int OPT_SOL      = 0x8;

    public static final boolean USE_TWIST_FLIP_PRUN = true;
    public static final boolean TRY_INVERSE = true;
    public static final boolean TRY_THREE_AXES = true;

    public static final boolean USE_COMBP_PRUN = USE_TWIST_FLIP_PRUN;
    public static final boolean USE_CONJ_PRUN  = USE_TWIST_FLIP_PRUN;

    public static final int MAX_PRE_MOVES = 20;
    public static final int MIN_P1LENGTH_PRE = 7;
    public static final int MAX_DEPTH2 = 12;
}
