package org.firstinspires.ftc.teamcode.anime;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.concurrent.CompletableFuture;

public class AnimeAction implements Action {
    protected Runnable runnable;
    private CompletableFuture<Void> actionTask;

    public AnimeAction(Runnable runnable) {
        this.runnable = runnable;

    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if(this.actionTask == null) {
            this.actionTask = CompletableFuture.runAsync(runnable);
        }
        return !this.actionTask.isDone();
    }

    public static AnimeAction createAction(Runnable task) {
        return new AnimeAction(task);
    }
}
