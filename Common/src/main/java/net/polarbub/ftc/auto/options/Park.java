package net.polarbub.ftc.auto.options;

public enum Park {
   YES,
    NO;

    @Override
    public String toString() {
        if(this == YES) {
            return "Park";
        } else if (this == NO) {
            return  "Do Not Park";
        }
        throw new RuntimeException("PARK toString() incomplete");
    }
}