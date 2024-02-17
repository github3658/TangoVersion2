package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj2.command.Command;

public class PlaySong extends Command {
    private final Orchestra o_Orchestra;
    private final String str_song;

    public PlaySong(Orchestra orchestra, String song) {
        o_Orchestra = orchestra;
        str_song = song;
    }

    @Override
    public void initialize() {
        o_Orchestra.loadMusic(str_song);
        o_Orchestra.play();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}