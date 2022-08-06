-module(map).

%%%=======================STATEMENT====================
-description("map").
-author("arthorn10086").

%%%=======================EXPORT=======================
-compile(export_all).

%%%=================EXPORTED FUNCTIONS=================
%% ----------------------------------------------------
%% @doc
%%        检测障碍物
%% @end
%% ----------------------------------------------------
check_point_barrier({X, Y, Z}) ->
    check_point_barrier(Z div 1000, xy2int(X, Y)).
check_point_barrier(MapType, Point) ->
    MapInfo = get_map_info(MapType),
    case ets:lookup(MapInfo, Point) of
        [] ->
            true;
        [{Point, 1}] ->
            true;
        [{Point, _Type}] ->
            false
    end.

%% ----------------------------------------------------
%% @doc
%%      xy坐标方法
%% @end
%% ----------------------------------------------------
int2xyz(Point) ->
    <<Z:16, X:16, Y:16>> = <<Point:48>>,
    {X, Y, Z}.
xyz2int({X, Y, Z}) ->
    xyz2int(X, Y, Z).
xyz2int(X, Y, Z) ->
    <<P:48>> = <<Z:16, X:16, Y:16>>,
    P.
xy2int({X, Y}) ->
    xy2int(X, Y).
xy2int(X, Y) ->
    <<P:32>> = <<X:16, Y:16>>,
    P.
%% ----------------------------------------------------
%% @doc
%%        获取地图的疆域
%% @end
%% ----------------------------------------------------
territory(1) ->
    {1, 10};
territory(2) ->
    {1, 100}.

%%%===================LOCAL FUNCTIONS==================
%% ----------------------------------------------------
%% @doc
%%      获取地图类型对应点配置
%% @end
%% ----------------------------------------------------
get_map_info(MapType) ->
    list_to_atom("map_info" ++ integer_to_list(MapType)).

%%%===================TEST FUNCTIONS==================
%% ----------------------------------------------------
%% @doc
%%  随机生成地图
%% @end
%% ----------------------------------------------------
random_init_map(MapType) ->
    {Min, Max} = territory(MapType),
    Border = lists:seq(Min, Max),
    Ets = get_map_info(MapType),
    L3 = [{xy2int(X, Y), rand:uniform(4) - 1} || X <- Border, Y <- Border],
    catch ets:new(Ets, [named_table, public, set, {read_concurrency, true}]),
    ets:insert(Ets, L3),
    show_map(MapType, []).

%% ----------------------------------------------------
%% @doc
%%  打印地图路线
%% @end
%% ----------------------------------------------------
astar_find(MapType, StartX, StartY, EndX, EndY, Opts) ->
    case astar:find({StartX, StartY, MapType * 1000}, {EndX, EndY, MapType * 1000}, Opts) of
        {ok, List} ->
            show_map(MapType, [xy2int(X, Y) || {X, Y, _} <- List]);
        Err ->
            Err
    end.
astar_valid_find(MapType, StartX, StartY, EndX, EndY, Opts) ->
    case astar:valid_find({StartX, StartY, MapType * 1000}, {EndX, EndY, MapType * 1000}, Opts) of
        {ok, List} ->
            show_map(MapType, [xy2int(X, Y) || {X, Y, _} <- List]);
        Err ->
            Err
    end.

%% ----------------------------------------------------
%% @doc
%%  打印地图   @行动路线 *障碍 +空地
%% @end
%% ----------------------------------------------------
show_map(MapType, PathList) ->
    {Min, Max} = territory(MapType),
    Border = lists:seq(Min, Max),
    PointL = [begin
        [{Point, Flag}] = ets:lookup(get_map_info(MapType), xy2int(X, Y)),
        {Point, Flag}
    end || Y <- Border, X <- Border],
    {_, Last, L} = lists:foldl(fun({P, F}, {Index, Acc, R}) ->
        Bool = lists:member(P, PathList),
        S = if
            Bool ->
                $@;
            F =:= 1 ->
                $*;
            true ->
                32
        end,
        if
            Index =:= Max ->
                {1, [32, S], [Acc | R]};
            true ->
                {Index + 1, [32, S | Acc], R}
        end
    end, {0, [], []}, PointL),
    [io:format(lists:reverse(LL) ++ "~n") || LL <- [Last | L]].

