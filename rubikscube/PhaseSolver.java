package rubikscube;

public class PhaseSolver {
    private static final int NUM_NODES = PhaseConstants.NUM_NODES;
    private static final int NUM_URF   = PhaseConstants.NUM_URF;

    public static final int USE_SEPARATOR = PhaseConstants.USE_SEPARATOR;
    public static final int INVERSE_SOL   = PhaseConstants.INVERSE_SOL;
    public static final int APPEND_LEN    = PhaseConstants.APPEND_LEN;
    public static final int OPT_SOL       = PhaseConstants.OPT_SOL;

    public static final boolean USE_TWIST_FLIP_PRUN = PhaseConstants.USE_TWIST_FLIP_PRUN;
    static final boolean TRY_INVERSE                = PhaseConstants.TRY_INVERSE;
    static final boolean TRY_THREE_AXES            = PhaseConstants.TRY_THREE_AXES;

    static final boolean USE_COMBP_PRUN = PhaseConstants.USE_COMBP_PRUN;
    static final boolean USE_CONJ_PRUN  = PhaseConstants.USE_CONJ_PRUN;

    private static final int MAX_PRE_MOVES      = PhaseConstants.MAX_PRE_MOVES;
    protected static int MIN_P1LENGTH_PRE       = PhaseConstants.MIN_P1LENGTH_PRE;
    protected static int MAX_DEPTH2             = PhaseConstants.MAX_DEPTH2;

    static boolean inited = false;

    protected long selfSym;
    protected int  maskConj;
    protected int  urfIdx;
    protected int  len1;
    protected int  depth1;
    protected int  maxDepth2;
    protected int  solLen;

    protected CubeMapping.Solution solu;

    protected long probe;
    protected long probeMax;
    protected long probeMin;

    protected int  verbose;
    protected int  valid1;
    protected boolean allowShorter = false;

    protected Cubie cc = new Cubie();

    protected Cubie[] urfCubieCube = new Cubie[NUM_URF];
    protected Coordinates[] urfCoordCube = new Coordinates[NUM_URF];

    protected Cubie[] phase1Cubie = new Cubie[NUM_NODES];
    protected Coordinates[] nodeUD      = new Coordinates[NUM_NODES];
    protected Coordinates[] nodeRL      = new Coordinates[NUM_NODES];
    protected Coordinates[] nodeFB      = new Coordinates[NUM_NODES];

    Cubie[] preMoveCubes = new Cubie[MAX_PRE_MOVES + 1];
    int[]   preMoves     = new int[MAX_PRE_MOVES];
    int     preMoveLen   = 0;
    int     maxPreMoves  = 0;

    protected boolean isRec = false;

    protected int[] move = new int[31];

    public PhaseSolver() {
        initNodes();
        initUrfStructures();
        initPreMoves();
    }

    private void initNodes() {
        for (int i = 0; i < NUM_NODES; i++) {
            nodeUD[i]      = new Coordinates();
            nodeRL[i]      = new Coordinates();
            nodeFB[i]      = new Coordinates();
            phase1Cubie[i] = new Cubie();
        }
    }

    private void initUrfStructures() {
        for (int i = 0; i < NUM_URF; i++) {
            urfCubieCube[i] = new Cubie();
            urfCoordCube[i] = new Coordinates();
        }
    }

    private void initPreMoves() {
        for (int i = 1; i <= MAX_PRE_MOVES; i++) {
            preMoveCubes[i] = new Cubie();
        }
    }

    public synchronized String solution(String facelets, int maxDepth, long probeMax, long probeMin, int verbose) {
        int check = verify(facelets);
        if (check != 0) {
            return "Error " + Math.abs(check);
        }

        resetSearchState(maxDepth, probeMax, probeMin, verbose);

        Coordinates.init(false);
        initSearch();

        boolean wantOptimal = (verbose & OPT_SOL) != 0;
        return wantOptimal ? searchopt() : search();
    }

    public synchronized String next(long probeMax, long probeMin, int verbose) {
        this.probe    = 0;
        this.probeMax = probeMax;
        this.probeMin = Math.min(probeMin, probeMax);

        boolean prevOptimal = (this.verbose & OPT_SOL) != 0;
        boolean newOptimal  = (verbose & OPT_SOL) != 0;

        this.isRec   = (prevOptimal == newOptimal);
        this.verbose = verbose;
        this.solu    = null;

        return newOptimal ? searchopt() : search();
    }

    public static boolean isInited() {
        return inited;
    }

    public long numProbes() {
        return probe;
    }

    public int length() {
        return solLen;
    }

    public synchronized static void init() {
        Coordinates.init(true);
        inited = true;
    }

    private void resetSearchState(int maxDepth, long probeMax, long probeMin, int verbose) {
        this.solLen   = maxDepth + 1;
        this.probe    = 0;
        this.probeMax = probeMax;
        this.probeMin = Math.min(probeMin, probeMax);
        this.verbose  = verbose;

        this.solu  = null;
        this.isRec = false;
    }

    protected void initSearch() {
        int conjMask = 0;
        if (!TRY_INVERSE) {
            conjMask |= 0x38;
        }
        if (!TRY_THREE_AXES) {
            conjMask |= 0x36;
        }
        maskConj = conjMask;

        selfSym = cc.selfSymmetry();

        if (((selfSym >>> 16) & 0xFFFFL) != 0L) {
            maskConj |= 0x12;
        }
        if (((selfSym >>> 32) & 0xFFFFL) != 0L) {
            maskConj |= 0x24;
        }
        if (((selfSym >>> 48) & 0xFFFFL) != 0L) {
            maskConj |= 0x38;
        }

        selfSym &= 0xFFFFFFFFFFFFL;

        maxPreMoves = (maskConj > 7) ? 0 : MAX_PRE_MOVES;

        for (int i = 0; i < NUM_URF; i++) {
            urfCubieCube[i].copy(cc);
            urfCoordCube[i].setPrun(urfCubieCube[i], 20);

            cc.URFConjugate();
            if (i % 3 == 2) {
                cc.invCubie();
            }
        }
    }

    int verify(String facelets) {
        byte[] mapped = new byte[54];
        int colorMask = 0;

        try {
            String centers =
                    "" + facelets.charAt(Facelets.U5) +
                    facelets.charAt(Facelets.R5) +
                    facelets.charAt(Facelets.F5) +
                    facelets.charAt(Facelets.D5) +
                    facelets.charAt(Facelets.L5) +
                    facelets.charAt(Facelets.B5);

            for (int i = 0; i < 54; i++) {
                char c = facelets.charAt(i);
                int colorIndex = centers.indexOf(c);
                if (colorIndex < 0) {
                    return -1;
                }
                mapped[i] = (byte) colorIndex;
                colorMask += 1 << (colorIndex * 4);
            }
        } catch (Exception e) {
            return -1;
        }

        //Each color must appear exactly 9 times.
        if (colorMask != 0x999999) {
            return -1;
        }

        CubeMapping.toCubieCube(mapped, cc);
        return cc.verify();
    }

    protected int phase1PreMoves(int remaining, int lastMove, Cubie curr, int symMask) {
        preMoveLen = maxPreMoves - remaining;

        boolean shouldSearch;
        if (isRec) {
            shouldSearch = (depth1 == len1 - preMoveLen);
        } else {
            boolean noPreMovesYet = (preMoveLen == 0);
            boolean lastMoveAllowed = ((0x36FB7 >> lastMove) & 1) == 0;
            shouldSearch = noPreMovesYet || lastMoveAllowed;
        }

        if (shouldSearch) {
            depth1 = len1 - preMoveLen;
            phase1Cubie[0] = curr;
            allowShorter = (depth1 == MIN_P1LENGTH_PRE && preMoveLen > 0);

            Coordinates startNode = nodeUD[depth1 + 1];
            if (startNode.setPrun(curr, depth1)) {
                int result = phase1(startNode, symMask, depth1, -1);
                if (result == 0) {
                    return 0;
                }
            }
        }

        if (remaining == 0 || preMoveLen + MIN_P1LENGTH_PRE >= len1) {
            return 1;
        }

        int skipMoves = Cubie.getSkipMoves(symMask);

        if (remaining == 1 || preMoveLen + 1 + MIN_P1LENGTH_PRE >= len1) {
            skipMoves |= 0x36FB7;
        }

        int lastAxisMove = (lastMove / 3) * 3;

        for (int m = 0; m < 18; m++) {
            if (m == lastAxisMove || m == lastAxisMove - 9 || m == lastAxisMove + 9) {
                m += 2;
                continue;
            }

            int idxPre = maxPreMoves - remaining;
            if ((isRec && m != preMoves[idxPre]) || ((skipMoves & (1 << m)) != 0)) {
                continue;
            }

            Cubie next = preMoveCubes[remaining];
            Cubie.CornMult(Cubie.moveCube[m], curr, next);
            Cubie.EdgeMult(Cubie.moveCube[m], curr, next);

            preMoves[idxPre] = m;

            int res = phase1PreMoves(remaining - 1, m, next, symMask & (int) Cubie.moveCubeSym[m]);

            if (res == 0) {
                return 0;
            }
        }

        return 1;
    }


    protected int phase1(Coordinates node, int ssym, int maxl, int lastAxis) {
        if (node.prun == 0 && maxl < 5) {
            if (allowShorter || maxl == 0) {
                int storedDepth = depth1;
                depth1 -= maxl;
                int ret = initPhase2Pre();
                depth1 = storedDepth;
                return ret;
            }
            return 1;
        }

        int skipMask = Cubie.getSkipMoves(ssym);
        final int targetDepth = depth1 - maxl;

        for (int axis = 0; axis < 18; axis += 3) {
            if (axis == lastAxis || axis == lastAxis - 9) {
                continue;
            }

            for (int power = 0; power < 3; power++) {
                int mv = axis + power;

                if (isRec && mv != move[targetDepth]) {
                    continue;
                }
                if ((skipMask & (1 << mv)) != 0) {
                    continue;
                }

                int pr = nodeUD[maxl].pruningMoves(node, mv, true);
                if (pr > maxl) {
                    break;  
                }
                if (pr == maxl) {
                    continue;
                }

                if (USE_CONJ_PRUN) {
                    int prConj = nodeUD[maxl].pruningMovesConj(node, mv);
                    if (prConj > maxl) {
                        break;
                    }
                    if (prConj == maxl) {
                        continue;
                    }
                }

                move[targetDepth] = mv;
                valid1 = Math.min(valid1, targetDepth);

                int newSym = ssym & (int) Cubie.moveCubeSym[mv];
                int res = phase1(nodeUD[maxl], newSym, maxl - 1, axis);

                if (res == 0) {
                    return 0;
                }
                if (res >= 2) {
                    break;
                }
            }
        }
        return 1;
    }


    protected String searchopt() {
        int maxPrunURF = 0;
        int maxPrunInv = 0;

        for (int i = 0; i < NUM_URF; i++) {
            urfCoordCube[i].calcPruning(false);
            if (i < 3) {
                maxPrunURF = Math.max(maxPrunURF, urfCoordCube[i].prun);
            } else {
                maxPrunInv = Math.max(maxPrunInv, urfCoordCube[i].prun);
            }
        }

        urfIdx = (maxPrunInv > maxPrunURF) ? 3 : 0;
        phase1Cubie[0] = urfCubieCube[urfIdx];

        for (len1 = isRec ? len1 : 0; len1 < solLen; len1++) {
            Coordinates ud = urfCoordCube[urfIdx];
            Coordinates rl = urfCoordCube[urfIdx + 1];
            Coordinates fb = urfCoordCube[urfIdx + 2];

            boolean prunable = (ud.prun <= len1) && (rl.prun <= len1) && (fb.prun <= len1);

            if (prunable) {
                int res = phase1opt(ud, rl, fb, selfSym, len1, -1);
                if (res == 0) {
                    return (solu == null) ? "Error " : solu.toString();
                }
            }
        }

        return (solu == null) ? "Error, no sol" : solu.toString();
    }

    protected int phase1opt(Coordinates ud, Coordinates rl, Coordinates fb, long ssym, int maxl, int lastAxis) {

        if (ud.prun == 0 && rl.prun == 0 && fb.prun == 0 && maxl < 5) {
            maxDepth2 = maxl;
            depth1 = len1 - maxl;
            return (initPhase2Pre() == 0) ? 0 : 1;
        }

        int skipMask = Cubie.getSkipMoves(ssym);
        final int targetIndex = len1 - maxl;

        for (int axis = 0; axis < 18; axis += 3) {
            if (axis == lastAxis || axis == lastAxis - 9) {
                continue;
            }

            for (int power = 0; power < 3; power++) {
                int mv = axis + power;

                if (isRec && mv != move[targetIndex]) {
                    continue;
                }
                if ((skipMask & (1 << mv)) != 0) {
                    continue;
                }

                // UD
                int prUD = nodeUD[maxl].pruningMoves(ud, mv, false);
                if (USE_CONJ_PRUN) {
                    prUD = Math.max(prUD,
                            nodeUD[maxl].pruningMovesConj(ud, mv));
                }
                if (prUD > maxl) {
                    break;
                }
                if (prUD == maxl) {
                    continue;
                }

                // RL
                int mvRL = Cubie.urfMove[2][mv];
                int prRL = nodeRL[maxl].pruningMoves(rl, mvRL, false);
                if (USE_CONJ_PRUN) {
                    prRL = Math.max(prRL, nodeRL[maxl].pruningMovesConj(rl, mvRL));
                }
                if (prRL > maxl) {
                    break;
                }
                if (prRL == maxl) {
                    continue;
                }

                // FB
                int mvFB = Cubie.urfMove[2][mvRL];
                int prFB = nodeFB[maxl].pruningMoves(fb, mvFB, false);
                if (USE_CONJ_PRUN) {
                    prFB = Math.max(prFB,
                            nodeFB[maxl].pruningMovesConj(fb, mvFB));
                }

                if (prUD == prRL && prRL == prFB && prFB != 0) {
                    prFB++; 
                }

                if (prFB > maxl) {
                    break;
                }
                if (prFB == maxl) {
                    continue;
                }

                int stdMove = Cubie.urfMove[2][mvFB];

                move[targetIndex] = stdMove;
                valid1 = Math.min(valid1, targetIndex);

                long newSymMask = ssym & Cubie.moveCubeSym[stdMove];
                int res = phase1opt(
                        nodeUD[maxl], nodeRL[maxl], nodeFB[maxl],
                        newSymMask, maxl - 1, axis);

                if (res == 0) {
                    return 0;
                }
            }
        }

        return 1;
    }


    protected String search() {
        for (len1 = isRec ? len1 : 0; len1 < solLen; len1++) {
            maxDepth2 = Math.min(MAX_DEPTH2, solLen - len1 - 1);

            for (urfIdx = isRec ? urfIdx : 0; urfIdx < NUM_URF; urfIdx++) {
                if ((maskConj & (1 << urfIdx)) != 0) {
                    continue;
                }

                int ok = phase1PreMoves(
                        maxPreMoves,
                        -30,
                        urfCubieCube[urfIdx],
                        (int) (selfSym & 0xFFFFL)
                );

                if (ok == 0) {
                    return (solu == null) ? "Error 8" : solu.toString();
                }
            }
        }
        return (solu == null) ? "Error 7" : solu.toString();
    }

    protected int initPhase2Pre() {
        isRec = false;

        long limit = (solu == null) ? probeMax : probeMin;
        if (probe >= limit) {
            return 0;
        }
        ++probe;

        for (int i = valid1; i < depth1; i++) {
            Cubie.CornMult(phase1Cubie[i], Cubie.moveCube[move[i]], phase1Cubie[i + 1]);
            Cubie.EdgeMult(phase1Cubie[i], Cubie.moveCube[move[i]], phase1Cubie[i + 1]);
        }
        valid1 = depth1;

        int p2corn = phase1Cubie[depth1].getCPermSym();
        int p2csym = p2corn & 0xF;
        p2corn >>>= 4;

        int p2edge = phase1Cubie[depth1].getEPermSym();
        int p2esym = p2edge & 0xF;
        p2edge >>>= 4;

        int p2mid = phase1Cubie[depth1].getMPerm();

        int edgei = Cubie.getPermSymInv(p2edge, p2esym, false);
        int corni = Cubie.getPermSymInv(p2corn, p2csym, true);

        int lastMove = (depth1 == 0)       ? -1 : move[depth1 - 1];
        int lastPre  = (preMoveLen == 0)   ? -1 : preMoves[preMoveLen - 1];

        int ret = 0;
        int p2switchMax = (preMoveLen == 0 ? 1 : 2) * (depth1 == 0 ? 1 : 2);
        int p2switchMask = (1 << p2switchMax) - 1;

        for (int p2switch = 0; p2switch < p2switchMax; p2switch++) {
            if ((p2switchMask & (1 << p2switch)) != 0) {
                p2switchMask &= ~(1 << p2switch);

                ret = initPhase2(p2corn, p2csym, p2edge, p2esym,
                                 p2mid, edgei, corni);
                if (ret == 0 || ret > 2) {
                    break;
                } else if (ret == 2) {
                    p2switchMask &= (0x4 << p2switch); 
                }
            }

            if (p2switchMask == 0) {
                break;
            }

            if ((p2switch & 1) == 0 && depth1 > 0) {
                int m = CubeMapping.std2ud[lastMove / 3 * 3 + 1];
                move[depth1 - 1] = CubeMapping.ud2std[m] * 2 - move[depth1 - 1];

                p2mid = Coordinates.MPermMove[p2mid][m];

                p2corn = Coordinates.CPermMove[p2corn][Cubie.SymMoveUD[p2csym][m]];
                p2csym = Cubie.SymMult[p2corn & 0xF][p2csym];
                p2corn >>>= 4;

                p2edge = Coordinates.EPermMove[p2edge][Cubie.SymMoveUD[p2esym][m]];
                p2esym = Cubie.SymMult[p2edge & 0xF][p2esym];
                p2edge >>>= 4;

                corni = Cubie.getPermSymInv(p2corn, p2csym, true);
                edgei = Cubie.getPermSymInv(p2edge, p2esym, false);

            } else if (preMoveLen > 0) {
                // adjust last pre-move
                int m = CubeMapping.std2ud[lastPre / 3 * 3 + 1];
                preMoves[preMoveLen - 1] = CubeMapping.ud2std[m] * 2 - preMoves[preMoveLen - 1];

                p2mid = Cubie.MPermInv[Coordinates.MPermMove[Cubie.MPermInv[p2mid]][m]];

                p2corn = Coordinates.CPermMove[corni >>> 4][Cubie.SymMoveUD[corni & 0xF][m]];
                corni  = (p2corn & ~0xF) | Cubie.SymMult[p2corn & 0xF][corni & 0xF];
                p2corn = Cubie.getPermSymInv(corni >>> 4, corni & 0xF, true);
                p2csym = p2corn & 0xF;
                p2corn >>>= 4;

                p2edge = Coordinates.EPermMove[edgei >>> 4][Cubie.SymMoveUD[edgei & 0xF][m]];
                edgei  = (p2edge & ~0xF) | Cubie.SymMult[p2edge & 0xF][edgei & 0xF];
                p2edge = Cubie.getPermSymInv(edgei >>> 4, edgei & 0xF, false);
                p2esym = p2edge & 0xF;
                p2edge >>>= 4;
            }
        }

        if (depth1 > 0) {
            move[depth1 - 1] = lastMove;
        }
        if (preMoveLen > 0) {
            preMoves[preMoveLen - 1] = lastPre;
        }

        return (ret == 0) ? 0 : 2;
    }

    protected int initPhase2(int p2corn, int p2csym, int p2edge, int p2esym, int p2mid, int edgei, int corni) {
        int pr1 = Coordinates.getPruning( Coordinates.EPermCCombPPrun, (edgei >> 4) * Constants.N_COMB + 
        Coordinates.CCombPConj[Cubie.Perm2CombP[corni >> 4] & 0xFF][Cubie.SymMultInv[edgei & 0xF][corni & 0xF]]);

        int pr2 = Coordinates.getPruning(Coordinates.EPermCCombPPrun, p2edge * Constants.N_COMB
                + Coordinates.CCombPConj[Cubie.Perm2CombP[p2corn] & 0xFF][Cubie.SymMultInv[p2esym][p2csym]]);

        int pr3 = Coordinates.getPruning(Coordinates.MCPermPrun,p2corn * Constants.N_MPERM
                + Coordinates.MPermConj[p2mid][p2csym]);

        int prun = Math.max(pr1, Math.max(pr2, pr3));

        if (prun > maxDepth2) {
            return prun - maxDepth2;
        }

        int depth2;
        for (depth2 = maxDepth2; depth2 >= prun; depth2--) {
            int ret = phase2(p2edge, p2esym, p2corn, p2csym, p2mid, depth2, depth1, 10);
            if (ret < 0) {
                break;
            }

            depth2 -= ret;
            solLen = 0;

            solu = new CubeMapping.Solution();
            solu.setArgs(verbose, urfIdx, depth1);

            for (int i = 0; i < depth1 + depth2; i++) {
                solu.appendSolMove(move[i]);
            }
            for (int i = preMoveLen - 1; i >= 0; i--) {
                solu.appendSolMove(preMoves[i]);
            }

            solLen = solu.length;
        }

        if (depth2 != maxDepth2) {
            maxDepth2 = Math.min(MAX_DEPTH2, solLen - len1 - 1);
            return (probe >= probeMin) ? 0 : 1;
        }
        return 1;
    }

    protected int phase2(int edge, int esym, int corn, int csym,int mid, int maxl, int depth, int lastMove) {
        if (edge == 0 && corn == 0 && mid == 0) {
            return maxl;
        }

        int moveMask = CubeMapping.ckmv2bit[lastMove];

        for (int m = 0; m < 10; m++) {
            // Skip disallowed next moves
            if (((moveMask >>> m) & 1) != 0) {
                m += (0x42 >>> m) & 3;
                continue;
            }

            int mid2  = Coordinates.MPermMove[mid][m];
            int corn2 = Coordinates.CPermMove[corn][Cubie.SymMoveUD[csym][m]];
            int csym2 = Cubie.SymMult[corn2 & 0xF][csym]; corn2 >>>= 4;
            int edge2 = Coordinates.EPermMove[edge][Cubie.SymMoveUD[esym][m]];
            int esym2 = Cubie.SymMult[edge2 & 0xF][esym]; edge2 >>>= 4;

            int edgeInv = Cubie.getPermSymInv(edge2, esym2, false);
            int cornInv = Cubie.getPermSymInv(corn2, csym2, true);

            int pr = Coordinates.getPruning(Coordinates.EPermCCombPPrun, (edgeInv >> 4) * Constants.N_COMB + 
            Coordinates.CCombPConj[Cubie.Perm2CombP[cornInv >> 4] & 0xFF][Cubie.SymMultInv[edgeInv & 0xF][cornInv & 0xF]]);

            if (pr > maxl + 1) {
                return maxl - pr + 1;
            }
            if (pr >= maxl) {
                m += (0x42 >>> m & 3) & (maxl - pr);
                continue;
            }

            int prA = Coordinates.getPruning(Coordinates.MCPermPrun, corn2 * Constants.N_MPERM + Coordinates.MPermConj[mid2][csym2]);
            int prB = Coordinates.getPruning(Coordinates.EPermCCombPPrun, edge2 * Constants.N_COMB + 
                Coordinates.CCombPConj[Cubie.Perm2CombP[corn2] & 0xFF][Cubie.SymMultInv[esym2][csym2]]);

            pr = Math.max(prA, prB);

            if (pr >= maxl) {
                m += (0x42 >>> m & 3) & (maxl - pr);
                continue;
            }

            int ret = phase2(edge2, esym2, corn2, csym2, mid2, maxl - 1, depth + 1, m);

            if (ret >= 0) {
                move[depth] = CubeMapping.ud2std[m];
                return ret;
            }

            if (ret < -2) {
                break;
            }
            if (ret < -1) {
                m += (0x42 >>> m) & 3;
            }
        }

        return -1;
    }
}
