#! /usr/bin/env ruby

require 'orocos'
require 'readline'
include Orocos


Orocos.initialize

Orocos.run 'telemetry_telecommand::Task' => 'telemetry_telecommand' do

    Orocos.log_all

    tmtc = TaskContext.get 'telemetry_telecommand'

    tmtc.configure
    tmtc.start

    #reader = vicon.pose_samples.reader

    #while true
    #    if sample = reader.read
	#        puts "%s %s %s" % [sample.position.x, sample.position.y, sample.position.z]
       # end
       # sleep 0.01
    # end
    Readline::readline("Press ENTER to exit\n") do
    end
    tmtc.stop
end
