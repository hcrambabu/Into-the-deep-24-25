package org.firstinspires.ftc.teamcode.anime;

import java.util.concurrent.CompletableFuture;

public class AsyncUtility {
    public static CompletableFuture<Void> createAsyncTask(CompletableFuture<Void> previous, Runnable task) {
        if(previous != null && !previous.isDone() && !previous.isCancelled()) {
            return previous;
        }
        return CompletableFuture.runAsync(task);
    }
}
