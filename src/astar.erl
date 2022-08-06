-module(astar).

%%%=======================STATEMENT====================
-description("astar").
-author("arthorn10086").

%%%=======================EXPORT=======================
-export([find/3, valid_find/3]).
-export([manhattan/2, euclid/2, diagonal/2]).
-export([make_path/2, make_valid_path/2]).
%%%=======================RECORD=======================
-record(map_node, {
    point :: {integer(), integer(), integer()},%{x,y,z}
    father = {-1, -1} :: {integer(), integer(), integer()},%父节点
    f = 0 :: float(),%起点到终点的最短路径长度    F=G+H
    g = 0 :: float(),%从起点到该节点的移动消耗
    h = 0 :: float()%该节点到终点的预计消耗
}
).
%%%=======================DEFINE=======================
-define(MAX_EXEC, 65535).
-define(NORMAL_PATH, 1).%%普通全路径
-define(JUMP_PATH, 2).%%拐点路径
-define(SMOOTH_PATH, 3).%%合并拐点拉直
-define(CHK_BARRIER(Point), map:check_point_barrier(Point)).
%%%=======================TYPE=======================
-type map_node() :: #map_node{}.
-type point() :: {X :: non_neg_integer(), Y :: non_neg_integer(), MapId :: non_neg_integer()}.
-type path_type() :: ?NORMAL_PATH|?JUMP_PATH|?SMOOTH_PATH.
-type option() ::
{'max_execute', pos_integer()}|
{'path_type', path_type()}|
{'barrier_punishment', non_neg_integer()}.
%% ----------------------------------------------------
%% @doc
%%      查找路径
%% @end
%% ----------------------------------------------------
-spec find(Start, End, Options) -> Err|{ok, PL} when
    Start :: point(),
    End :: point(),
    Options :: [option()],
    Err :: string(),
    PL :: [point()].
find(Start, End, Options) ->
    %%检查终点障碍物
    Bool = ?CHK_BARRIER(End),
    if
        Bool ->
            "unable_to_reach";
        true ->
            MaxExec = proplists:get_value('max_execute', Options, 65535),
            put('end_point', End),
            StartNode = #map_node{point = Start},
            CloseList = #{Start => -1},
            OpenList = gb_trees:empty(),
            OpenList1 = gb_trees:insert({StartNode#map_node.f, Start}, StartNode, OpenList),
            case start_find(OpenList1, CloseList, MaxExec) of
                Err when is_list(Err) ->
                    Err;
                Node ->
                    PathType = proplists:get_value('path_type', Options, ?NORMAL_PATH),
                    make_path(Node, PathType)
            end
    end.

%% ----------------------------------------------------
%% @doc
%%      查找路径  目标不可达时，尽可能往终点靠近
%% @end
%% ----------------------------------------------------
-spec valid_find(Start, End, Options) -> Err|{ok, PL} when
    Start :: point(),
    End :: point(),
    Options :: [option()],
    Err :: string(),
    PL :: [point()].
valid_find(Start, End, Options) ->
    MaxExec = proplists:get_value('max_execute', Options, 65535),
    BarrierAdd = proplists:get_value('barrier_punishment', Options, 9999),
    put('end_point', End),
    StartNode = #map_node{point = Start},
    CloseList = #{Start => -1},
    OpenList = gb_trees:empty(),
    OpenList1 = gb_trees:insert({StartNode#map_node.f, Start}, StartNode, OpenList),
    case start_valid_find(OpenList1, CloseList, MaxExec, BarrierAdd) of
        Err when is_list(Err) ->
            Err;
        Node ->
            PathType = proplists:get_value('path_type', Options, ?NORMAL_PATH),
            make_valid_path(Node, PathType)
    end.

%% ----------------------------------------------------
%% @doc
%%
%% @end
%% ----------------------------------------------------
-spec start_valid_find(OpenList, CloseList, Count, BarrierAdd) -> Err|Node when
    OpenList :: gb_trees:tree(),
    CloseList :: map(),
    Count :: non_neg_integer(),
    BarrierAdd :: non_neg_integer(),
    Err :: string(),
    Node :: map_node().
start_valid_find({0, nil}, _, _, _) -> "not_find";
start_valid_find(_, _, 0, _) -> "max_limit";
start_valid_find(OpenList, CloseList, Count, BarrierAdd) ->
    %%在开放列表里面找到最佳节点
    {_, NextNode, OpenListT} = gb_trees:take_smallest(OpenList),
    case NextNode#map_node.point =:= get('end_point') of
        true ->%%终点
            NextNode;
        false ->
            %%更新closelist
            NewCloseList = CloseList#{NextNode#map_node.point => -1},
            put(NextNode#map_node.point, NextNode),
            %%
            Points = get_neighbors1(NextNode, NewCloseList),
            %%更新openlist
            {NewOpenList, NewCloseList1} = update_valid_open_list(OpenListT, NewCloseList, NextNode, Points, BarrierAdd),
            start_valid_find(NewOpenList, NewCloseList1, Count - 1, BarrierAdd)
    end.

%% ----------------------------------------------------
%% @doc
%%
%% @end
%% ----------------------------------------------------
-spec start_find(OpenList, CloseList, Count) -> Err|Node when
    OpenList :: gb_trees:tree(),
    CloseList :: map(),
    Count :: non_neg_integer(),
    Err :: string(),
    Node :: map_node().
start_find({0, nil}, _, _) -> "not_find";
start_find(_, _, 0) -> "max_limit";
start_find(OpenList, CloseList, Count) ->
    %%在开放列表里面找到最佳节点
    {_, NextNode, OpenListT} = gb_trees:take_smallest(OpenList),
    case NextNode#map_node.point =:= get('end_point') of
        true ->%%终点
            NextNode;
        false ->
            %%更新closelist
            NewCloseList = CloseList#{NextNode#map_node.point => -1},
            put(NextNode#map_node.point, NextNode),
            %%
            Points = get_neighbors(NextNode, NewCloseList),
            %%更新openlist
            {NewOpenList, NewCloseList1} = update_open_list(OpenListT, NewCloseList, NextNode, Points),
            start_find(NewOpenList, NewCloseList1, Count - 1)
    end.

%% ----------------------------------------------------
%% @doc
%%  更新open_list
%% @end
%% ----------------------------------------------------
-spec update_open_list(OpenList, CloseList, NextNode, Points) -> {NewOpenList, NewCloseList} when
    OpenList :: gb_trees:tree(),
    CloseList :: map(),
    NextNode :: map_node(),
    Points :: [point()],
    NewOpenList :: gb_trees:tree(),
    NewCloseList :: map().
update_open_list(OpenList, CloseList, NextNode, Points) ->
    %%找到周围的点可以通行的点
    lists:foldl(fun(Point, {Acc1, Acc2}) ->
        NewNode = init_map_node(Point, NextNode),
        Score = NewNode#map_node.f,
        case maps:find(Point, CloseList) of
            error ->%%第一次进openlist
                {gb_trees:insert({Score, Point}, NewNode, Acc1), Acc2#{Point => Score}};
            {ok, OldScore} when OldScore =< Score ->
                {Acc1, Acc2};
            {ok, OldScore} ->
                NAcc1 = gb_trees:insert({Score, Point}, NewNode, gb_trees:delete({OldScore, Point}, Acc1)),
                {NAcc1, Acc2#{Point => Score}}
        end
    end, {OpenList, CloseList}, Points).

%% ----------------------------------------------------
%% @doc
%%  更新open_list
%% @end
%% ----------------------------------------------------
-spec update_valid_open_list(OpenList, CloseList, NextNode, Points, BarrierAdd) -> {NewOpenList, NewCloseList} when
    OpenList :: gb_trees:tree(),
    CloseList :: map(),
    NextNode :: map_node(),
    Points :: [point()],
    BarrierAdd :: non_neg_integer(),
    NewOpenList :: gb_trees:tree(),
    NewCloseList :: map().
update_valid_open_list(OpenList, CloseList, NextNode, Points, BarrierAdd) ->
    %%找到周围的点可以通行的点
    lists:foldl(fun(Point, {Acc1, Acc2}) ->
        NewNode = init_map_node1(Point, NextNode, BarrierAdd),
        Score = NewNode#map_node.f,
        case maps:find(Point, CloseList) of
            error ->%%第一次进openlist
                {gb_trees:insert({Score, Point}, NewNode, Acc1), Acc2#{Point => Score}};
            {ok, OldScore} when OldScore =< Score ->
                {Acc1, Acc2};
            {ok, OldScore} ->
                NAcc1 = gb_trees:insert({Score, Point}, NewNode, gb_trees:delete({OldScore, Point}, Acc1)),
                {NAcc1, Acc2#{Point => Score}}
        end
    end, {OpenList, CloseList}, Points).

%% ----------------------------------------------------
%% @doc
%%  生成路径
%% @end
%% ----------------------------------------------------
-spec make_path(Node, PathType) -> Path when
    Node :: map_node(),
    PathType :: path_type(),
    Path :: [point()].
make_path(Node, ?NORMAL_PATH) ->
    make_normal_path(Node, []);
make_path(Node, ?JUMP_PATH) ->
    {ok, PL} = make_normal_path(Node, []),
    make_jump_path(PL);
make_path(Node, ?SMOOTH_PATH) ->
    {ok, L1} = make_normal_path(Node, []),
    {ok, L} = make_jump_path(L1),
    merge_jump_point(L).


%% ----------------------------------------------------
%% @doc
%%      追溯生成路线
%% @end
%% ----------------------------------------------------
-spec make_normal_path(undefined|map_node(), [point()]) -> {ok, [point()]}.
make_normal_path(undefined, Path) ->
    {ok, Path};
make_normal_path(Node, Path) ->
    Father = Node#map_node.father,
    Point = Node#map_node.point,
    FNode = get(Father),
    make_normal_path(FNode, [Point | Path]).
%% ----------------------------------------------------
%% @doc
%%      追溯生成路线 有障碍物
%% @end
%% ----------------------------------------------------
-spec make_valid_normal_path(undefined|map_node(), [point()]) -> {ok, [point()]}.
make_valid_normal_path(undefined, Path) ->
    {ok, Path};
make_valid_normal_path(Node, Path) ->
    Father = Node#map_node.father,
    Point = Node#map_node.point,
    FNode = get(Father),
    case ?CHK_BARRIER(Point) of
        true ->
            make_valid_normal_path(FNode, []);
        false ->
            make_valid_normal_path(FNode, [Point | Path])
    end.
%% ----------------------------------------------------
%% @doc
%%      删除共线点，保留拐点
%% @end
%% ----------------------------------------------------
-spec make_jump_path([point()]) -> {ok, [point()]}.
make_jump_path([P1, P2 | L]) ->
    D = go_direction(P2, P1),
    make_jump_path(L, D, P2, [P1]);
make_jump_path(P) ->
    {ok, P}.

make_jump_path([], _Direction, Point, List) ->
    {ok, [Point | List]};
make_jump_path([Point | L], Direction, LastPoint, List) ->
    case go_direction(Point, LastPoint) of
        Direction ->%%同线，并点
            make_jump_path(L, Direction, Point, List);
        D1 ->%%拐点  把上一个点加进路径列表
            make_jump_path(L, D1, Point, [LastPoint | List])
    end.

%% ----------------------------------------------------
%% @doc
%%  生成路径
%% @end
%% ----------------------------------------------------
-spec make_valid_path(map_node(), path_type()) -> {ok, [point()]}.
make_valid_path(Node, ?NORMAL_PATH) ->
    make_valid_normal_path(Node, []);
make_valid_path(Node, ?JUMP_PATH) ->
    {ok, L} = make_valid_normal_path(Node, []),
    make_jump_path(L);
make_valid_path(Node, ?SMOOTH_PATH) ->
    {ok, L1} = make_valid_normal_path(Node, []),
    {ok, L} = make_jump_path(L1),
    merge_jump_point(L).


%% ----------------------------------------------------
%% @doc
%%
%% @end
%% ----------------------------------------------------
-spec init_map_node(point(), map_node()) -> map_node().
init_map_node({X, Y, Z}, NextNode) ->
    {X1, Y1, _} = NextNode#map_node.point,
    Consume = diagonal({X, Y}, {X1, Y1}),
    G = NextNode#map_node.g + Consume,
    {X2, Y2, _} = get('end_point'),
    H = diagonal({X, Y}, {X2, Y2}),
    #map_node{point = {X, Y, Z}, father = {X1, Y1, Z}, f = G + H, g = G, h = H}.
%% ----------------------------------------------------
%% @doc
%%
%% @end
%% ----------------------------------------------------
-spec init_map_node1(point(), map_node(), non_neg_integer()) -> map_node().
init_map_node1({X, Y, Z}, NextNode, BarrierAdd) ->
    {X1, Y1, Z1} = NextNode#map_node.point,
    Consume = case ?CHK_BARRIER({X, Y, Z}) of
        true ->
            case ?CHK_BARRIER({X1, Y1, Z1}) of
                true ->
                    diagonal({X, Y}, {X1, Y1}) + BarrierAdd/2;
                false ->
                    diagonal({X, Y}, {X1, Y1}) + BarrierAdd
            end;
        false ->
            diagonal({X, Y}, {X1, Y1})
    end,
    G = NextNode#map_node.g + Consume,
    {X2, Y2, _} = get('end_point'),
    H = diagonal({X, Y}, {X2, Y2}),
    #map_node{point = {X, Y, Z}, father = {X1, Y1, Z}, f = G + H, g = G, h = H}.

%% ----------------------------------------------------
%% @doc
%%      1.不是障碍物 2.不在closelist 3.不是障碍物
%% @end
%% ----------------------------------------------------
-spec get_neighbors(map_node(), map()) -> [point()].
get_neighbors(NextNode, CloseList) ->
    {X, Y, Z} = NextNode#map_node.point,
    F = fun(I, J) ->
        (not maps:is_key({I, J, Z}, CloseList) orelse maps:find({I, J, Z}, CloseList) > {ok, -1}) andalso
            (not ?CHK_BARRIER({I, J, Z}))
    end,
    [{I1, I2, Z} || {I1, I2} <- [{X - 1, Y - 1}, {X - 1, Y}, {X - 1, Y + 1}, {X, Y - 1}, {X, Y + 1}, {X + 1, Y - 1}, {X + 1, Y}, {X + 1, Y + 1}], F(I1, I2)].
%% ----------------------------------------------------
%% @doc
%%      1.不是障碍物 2.不在closelist
%% @end
%% ----------------------------------------------------
-spec get_neighbors1(map_node(), map()) -> [point()].
get_neighbors1(NextNode, CloseList) ->
    {X, Y, Z} = NextNode#map_node.point,
    F = fun(I, J) ->
        (not maps:is_key({I, J, Z}, CloseList) orelse maps:find({I, J, Z}, CloseList) > {ok, -1})
    end,
    [{I1, I2, Z} || {I1, I2} <- [{X - 1, Y - 1}, {X - 1, Y}, {X - 1, Y + 1}, {X, Y - 1}, {X, Y + 1}, {X + 1, Y - 1}, {X + 1, Y}, {X + 1, Y + 1}], F(I1, I2)].
%% ----------------------------------------------------
%% @doc
%%      H算法
%% @end
%% ----------------------------------------------------
manhattan({X, Y}, {X1, Y1}) ->
    erlang:abs(X1 - X) + erlang:abs(Y1 - Y).
euclid({X, Y}, {X1, Y1}) ->
    math:sqrt(math:pow(X1 - X, 2) + math:pow(Y1 - Y, 2)).
diagonal({X, Y}, {X1, Y1}) ->
    Dx = abs(X - X1),
    Dz = abs(Y - Y1),
    MinD = min(Dx, Dz),
    MinD * 1.414 + Dx + Dz - 2 * MinD.


%% ----------------------------------------------------
%% @doc
%%      合并拐点
%% @end
%% ----------------------------------------------------
-spec merge_jump_point([point()]) -> {ok, [point()]}.
merge_jump_point([P1, P2 | Path]) ->
    merge_jump_point(Path, P2, P1, [P1]).
merge_jump_point([], P, _SP, L) ->
    {ok, [P | L]};
merge_jump_point([CheckP | CheckL], P, SP, PathL) ->
    %%判断两点之间是否能直接走，计算点经过的所有格子
    CrossBool = check_cross(CheckP, SP),
    if
        CrossBool ->%%两点畅通，去除中间拐点
            case CheckL of
                [] ->%%CheckP是终点，结束，直接放进路径
                    {ok, [CheckP | PathL]};
                _ ->
                    merge_jump_point(CheckL, CheckP, SP, PathL)
            end;
        true ->%%不畅通，P点必须保留，加入路径列表,并作为下次判断的起点
            case CheckL of
                [] ->%%CheckP是终点，结束，直接放进路径
                    {ok, [CheckP, P | PathL]};
                _ ->
                    merge_jump_point(CheckL, CheckP, P, [P | PathL])
            end
    end.

%% ----------------------------------------------------
%% @doc
%%      判断两点直线是否畅通
%% @end
%% ----------------------------------------------------
-spec check_cross(point(), point()) -> boolean().
check_cross(CheckP, SP) ->
    {X1, Y1, MapId} = CheckP,
    {X2, Y2, _} = SP,
    if
        X1 =:= X2 ->%%垂直
            fory(X1, Y1, Y2, MapId, fun(X, Y, Z) ->
                ?CHK_BARRIER({X, Y, Z})
            end);
        Y1 =:= Y2 ->%%水平
            forx(Y1, X1, X2, MapId, fun(X, Y, Z) ->
                ?CHK_BARRIER({X, Y, Z})
            end);
        true ->%%斜
            %%斜截式  y = kx + b
            K = (Y2 - Y1)/(X2 - X1),
            B = Y2-K * X2,
            case erlang:abs(K) > 1 of
                true ->%%斜率大于1，沿Y轴查询
                    fory(X1, Y1, Y2, MapId, fun(_X, Y, Z) ->
                        ?CHK_BARRIER({ceil((Y - B) / K), Y, Z}) orelse ?CHK_BARRIER({trunc((Y - B) / K), Y, Z})
                    end);
                false ->%%斜率小于等于1，沿X轴查询
                    forx(Y1, X1, X2, MapId, fun(X, _Y, Z) ->
                        ?CHK_BARRIER({X, ceil(K * X + B), Z}) orelse ?CHK_BARRIER({X, trunc(K * X + B), Z})
                    end)
            end
    end.


%% ----------------------------------------------------
%% @doc
%%      遍历线段点
%% @end
%% ----------------------------------------------------
forx(_Y, EX, EX, _MapId, _CF) ->
    true;
forx(Y, X, EX, MapId, CF) ->
    case CF(X, Y, MapId) of
        true ->
            false;
        false ->
            forx(Y, X + 1, EX, MapId, CF)
    end.
fory(_X, EY, EY, _MapId, _CF) ->
    true;
fory(X, Y, EY, MapId, CF) ->
    case CF(X, Y, MapId) of
        true ->
            false;
        false ->
            fory(X, Y + 1, EY, MapId, CF)
    end.

%% ----------------------------------------------------
%% @doc
%%      行进方向
%% @end
%% ----------------------------------------------------
go_direction({X1, Y1, _}, {X2, Y2, _}) when X1 =:= X2 ->%%垂直  上/下
    {0, Y2 - Y1};
go_direction({X1, Y1, _}, {X2, Y2, _}) when Y1 =:= Y2 ->%%水平  左/右
    {X2 - X1, 0};
go_direction({X1, Y1, _}, {X2, Y2, _}) ->%%对角线
    {X2 - X1, Y2 - Y1}.
%% ----------------------------------------------------
%% @doc
%%      向上取整
%% @end
%% ----------------------------------------------------
ceil(Number) ->
    Int = trunc(Number),
    if
        Number > Int andalso Number > 0 ->
            Int + 1;
        true ->
            Int
    end.
