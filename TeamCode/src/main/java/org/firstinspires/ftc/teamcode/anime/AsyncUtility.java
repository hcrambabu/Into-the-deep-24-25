package org.firstinspires.ftc.teamcode.anime;

import java.util.concurrent.CompletableFuture;

public class AsyncUtility {
    public static CompletableFuture<Void> createAsyncTask(CompletableFuture<Void> previous, Runnable task) {
        if(isTaskNotDone(previous)) {
            return previous;
        }
        return CompletableFuture.runAsync(task);
    }

    public static boolean isTaskNotDone(CompletableFuture<Void> task) {
        return task != null && !task.isDone() && !task.isCancelled();
    }

    public static boolean isTaskDone(CompletableFuture<Void> task) {
        return !isTaskNotDone(task);
    }
}
