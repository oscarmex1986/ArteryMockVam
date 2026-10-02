//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package artery.application;

simple ExampleService like ItsG5Service
{
	parameters:
        @signal[ExaReceived](type=CaObject);
        @signal[ExaSent](type=CaObject);
        //@signal[SentExa](type=unsigned long);

        @statistic[reception](source=ExaReceived;record=vector(camStationId)?,vector(camGenerationDeltaTime)?);
        @statistic[transmission](source=ExaSent;record=vector(camStationId)?,vector(camGenerationDeltaTime)?);
        //@statistic[SentExa](title="SentExa"; source="SentExa";record=vector);
        // evaluate DCC transmission interval restrictions
        bool withDccRestriction = default(false); //OAM Default: true

        // generation interval boundaries
        double minInterval @unit(s) = default(0.1 s);
        double maxInterval @unit(s) = default(5.0 s); //OAM Default: 1.0
        double meanInterval @unit(s) = default(1.0 s);

        // generate at fixed rate (using minInterval, optionally restricted by DCC)
        bool fixedRate = default(false); //OAM Valor original "false"

        // OAM if true, transport type is GBC. If false, transport type is SHB
        bool multiHop = default(false); //OAM Valor original "false"

        // change in orientation triggering CAM generation (in degree)
        double headingDelta = default(4.0);

        // change in position triggering CAM generation
        double positionDelta @unit(m) = default(4.0m);

        // change in speed triggering CAM generation (in meter/second)
        double speedDelta @unit(mps) = default(0.5mps);

        // length of path history
        volatile int pathHistoryLength = default(23);
	
}