public static void main(String[] args) {
    ArrayList<Rot> baseRots = generateRotsList(); //We share 
    int[][] indexes = genIdxPairs(); 
    BiFunction<Rot, Rot, Rot> applyTo = (rot1, rot2) -> rot1.applyTo(rot2);
    Function<Rot, Rot> printResult = (rot1) -> {
        MRotation r = rot1.rotation;
        System.out.println("( "+r.getQ1()+", "+r.getQ2()+", "+r.getQ3()+", "+ r.getQ0()+")");
        return new Rot();
    };
    ArrayList<Rot> appliedTest = lambdaOnManyPairs(baseRots, indexes, applyTo);
    
}

/**
    * returns an array of results
    */
public static ArrayList<Rot> lambdaOnManyPairs(ArrayList<Rot> inputRots, int[][]idxs, BiFunction<Rot, Rot, Rot> func) {
    ArrayList<Rot> results = new ArrayList<Rot>();
    for(int i=0; i<idxs.length; i++) {
        int[] pair = idxs[i]; 
        results.add(lambdaOnPair(inputRots, pair[0], pair[1], func));
    }
    return results;
}

public static Rot lambdaOnPair(ArrayList<Rot> rotArr, int idx1, int idx2, BiFunction<Rot, Rot, Rot> func) {
    return func.apply(rotArr.get(idx1), rotArr.get(idx2));
}