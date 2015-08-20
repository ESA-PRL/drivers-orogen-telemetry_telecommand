#! /usr/bin/env ruby

require 'orocos'
require 'readline'
include Orocos


Orocos.initialize

Orocos.run 'data_handling::Task' => 'data_handling' do

    Orocos.log_all

    dh = TaskContext.get 'data_handling'

    dh.configure
    dh.start

    #reader = vicon.pose_samples.reader

    #while true
    #    if sample = reader.read
	#        puts "%s %s %s" % [sample.position.x, sample.position.y, sample.position.z]
       # end
       # sleep 0.01
    # end
    Readline::readline("Press ENTER to exit\n") do
    end
    dh.stop
end
