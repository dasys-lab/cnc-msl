/**
  This file is part of the MSL Refbox.

  The MSL Refbox is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License.

  The MSL Refbox is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with the MSL Refbox.  If not, see <http://www.gnu.org/licenses/>.
 */

package org.robocup.msl.refbox.protocol;

import org.robocup.msl.refbox.RefboxStatus;
import org.robocup.msl.refbox.constants.Team;
import org.robocup.msl.refbox.data.CardData;
import org.robocup.msl.refbox.data.GoalData;
import org.robocup.msl.refbox.data.PlayerChangeData;
import org.robocup.msl.refbox.data.TeamSetupData;
import org.robocup.msl.refbox.protocol2010.TeamColor;

public class Protocol2009 extends Protocol {

	public static final char COMM_STOP = 'S';

	public static final char COMM_START = 's';

	// game flow commands
	public static final char COMM_FIRST_HALF = '1';

	public static final char COMM_HALF_TIME = 'h';

	public static final char COMM_SECOND_HALF = '2';

	public static final char COMM_END_GAME = 'e';

	public static final char COMM_PARKING = 'L';

	public static final char COMM_CANCEL = 'x';

	// goal status
	public static final char COMM_GOAL_MAGENTA = 'a';

	public static final char COMM_GOAL_CYAN = 'A';

	public static final char COMM_SUBGOAL_MAGENTA = 'd';

	public static final char COMM_SUBGOAL_CYAN = 'D';

	/* game flow commands */
	public static final char COMM_KICKOFF_MAGENTA = 'k';

	public static final char COMM_KICKOFF_CYAN = 'K';

	public static final char COMM_FREEKICK_MAGENTA = 'f';

	public static final char COMM_FREEKICK_CYAN = 'F';

	public static final char COMM_GOALKICK_MAGENTA = 'g';

	public static final char COMM_GOALKICK_CYAN = 'G';

	public static final char COMM_THROWIN_MAGENTA = 't';

	public static final char COMM_THROWIN_CYAN = 'T';

	public static final char COMM_CORNER_MAGENTA = 'c';

	public static final char COMM_CORNER_CYAN = 'C';

	public static final char COMM_PENALTY_MAGENTA = 'p';

	public static final char COMM_PENALTY_CYAN = 'P';

	public static final char COMM_DROPPED_BALL = 'N';

	/* TODO: Bernd New Repair Timeout Commands */
	public static final char COMM_REPAIR_OUT_MAGENTA = 'o';

	public static final char COMM_REPAIR_OUT_CYAN = 'O';

	public static final char COMM_REPAIR_IN_MAGENTA = 'i';

	public static final char COMM_REPAIR_IN_CYAN = 'I';

	public static final String COMM_WELCOME_STRING = "Welcome..";

	public static final String COMM_RECONNECT_STRING = "Reconnect";
	
	
    private char chooseTeam(final Team team, final char cyanCommand, final char magentaCommand) {
        if (team == Team.TEAM_B) {
            return magentaCommand; /* magenta team */
        }
        return cyanCommand; /* CYAN team */
    }

    @Override
	protected final String cancel(final RefboxStatus refboxStatus) {
		return "" + Protocol2009.COMM_CANCEL;
	}

	@Override
	protected final String corner(final Team team, final RefboxStatus refboxStatus) {
		return "" + chooseTeam(team, Protocol2009.COMM_CORNER_CYAN, Protocol2009.COMM_CORNER_MAGENTA);
	}

	@Override
	protected final String droppedBall(final RefboxStatus refboxStatus) {
		return "" + Protocol2009.COMM_DROPPED_BALL;
	}

	@Override
	protected final String endGame(final RefboxStatus refboxStatus) {
		return "" + Protocol2009.COMM_END_GAME;
	}

	@Override
	protected final String firstHalfStart(final RefboxStatus refboxStatus) {
		return "" + Protocol2009.COMM_FIRST_HALF;
	}

	@Override
	protected final String freekick(final Team team, final RefboxStatus refboxStatus) {
		return "" + chooseTeam(team, Protocol2009.COMM_FREEKICK_CYAN, Protocol2009.COMM_FREEKICK_MAGENTA);
	}

	@Override
	protected final String goalkick(final Team team, final RefboxStatus refboxStatus) {
		return "" + chooseTeam(team, Protocol2009.COMM_GOALKICK_CYAN, Protocol2009.COMM_GOALKICK_MAGENTA);
	}

	@Override
	protected final String halfTimeStart(final RefboxStatus refboxStatus) {
		return "" + Protocol2009.COMM_HALF_TIME;
	}

	@Override
	protected final String kickoff(final Team team, final RefboxStatus refboxStatus) {
		return "" + chooseTeam(team, Protocol2009.COMM_KICKOFF_CYAN, Protocol2009.COMM_KICKOFF_MAGENTA);
	}

	@Override
	protected final String penalty(final Team team, final String goalColor, final RefboxStatus refboxStatus) {
		// goalColor is not used in protocol
		return "" + chooseTeam(team, Protocol2009.COMM_PENALTY_CYAN, Protocol2009.COMM_PENALTY_MAGENTA);
	}

	@Override
	protected final String reconnect() {
		return Protocol2009.COMM_RECONNECT_STRING;
	}

	@Override
	protected final String secondHalfStart(final RefboxStatus refboxStatus) {
		return "" + Protocol2009.COMM_SECOND_HALF;
	}

	@Override
	protected final String start(final RefboxStatus refboxStatus) {
		return "" + Protocol2009.COMM_START;
	}

	@Override
	protected final String stop(final RefboxStatus refboxStatus) {
		return "" + Protocol2009.COMM_STOP;
	}

	@Override
	protected final String throwin(final Team team, final RefboxStatus refboxStatus) {
		return "" + chooseTeam(team, Protocol2009.COMM_THROWIN_CYAN, Protocol2009.COMM_THROWIN_MAGENTA);
	}

	@Override
	protected final String welcome() {
		return Protocol2009.COMM_WELCOME_STRING;
	}

	@Override
	protected final String cardAwarded(final Team team, final CardData cardData, final RefboxStatus refboxStatus) {
		// not supported by this protocol
		return null;
	}

	@Override
	protected final String cardRemoved(final Team team, final CardData cardData, final RefboxStatus refboxStatus) {
		// not supported by this protocol
		return null;
	}

	@Override
	protected final String goalRemoved(final Team team, final GoalData goalData, final RefboxStatus refboxStatus) {
		return "" + chooseTeam(team, Protocol2009.COMM_SUBGOAL_CYAN, Protocol2009.COMM_SUBGOAL_MAGENTA);
	}

	@Override
	protected final String goalScored(final Team team, final GoalData goalData, final RefboxStatus refboxStatus) {
		return "" + chooseTeam(team, Protocol2009.COMM_GOAL_CYAN, Protocol2009.COMM_GOAL_MAGENTA);
	}

	@Override
	protected final String playerIn(final Team team, final PlayerChangeData data, final RefboxStatus refboxStatus) {
		// not supported by this protocol
		return null;
	}

	@Override
	protected final String playerOut(final Team team, final PlayerChangeData data, final RefboxStatus refboxStatus) {
		// not supported by this protocol
		return null;
	}

	@Override
	protected final String substitution(final Team team, final PlayerChangeData data, final RefboxStatus refboxStatus) {
		// not supported by this protocol
		return null;
	}

	@Override
	protected final String repairTimeout(final Team team, final Boolean in, final RefboxStatus refboxStatus) {
		if (in) {
			return "" + chooseTeam(team, Protocol2009.COMM_REPAIR_IN_CYAN, Protocol2009.COMM_REPAIR_IN_MAGENTA);
		} else {
			return "" + chooseTeam(team, Protocol2009.COMM_REPAIR_OUT_CYAN, Protocol2009.COMM_REPAIR_OUT_MAGENTA);
		}
	}

	@Override
	protected final String getGameInfo(final RefboxStatus refboxStatus) {
		// not supported by this protocol
		return null;
	}

	@Override
	protected final String getTeamInfo(final String teamName, final TeamSetupData teamSetupData) {
		// not supported by this protocol
		return null;
	}

	@Override
	public final Team getTeamType(final TeamColor teamColor, final RefboxStatus refboxStatus) {
		// not supported by this protocol
		return null;
	}

	@Override
	protected final String parking(final RefboxStatus refboxStatus) {
		return "" + Protocol2009.COMM_PARKING;
	}

}
