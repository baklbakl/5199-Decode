package org.ducktapeandaprayer.ftc.lib;

@FunctionalInterface
public interface QuadConsumer<T, U, V, W> {

    /**
     * Performs the operation given the specified arguments.
     *
     * @param k the first input argument
     * @param v the second input argument
     * @param s the third input argument
     */
    void accept(T k, U v, V s, W x);
}
