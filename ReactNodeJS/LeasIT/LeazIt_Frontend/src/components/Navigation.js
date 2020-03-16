import React from 'react';
import '../scss/custom.css';
import logo from '../images/logo.png';
import AuthService from "../services/AuthService";
import { MenuButton, ListItem, Avatar, FontIcon, Button } from 'react-md';
// import { Button } from 'reactstrap';
import {withRouter} from 'react-router-dom';
import KebabMenu from './KebabMenu';

const NavItem = props => {
    const pageURI = window.location.pathname+window.location.search;

    const liClassName = (props.path === pageURI) ? "nav-item active" : "nav-item";
    const aClassName = props.disabled ? "nav-link disabled" : "nav-link";
    // console.log("path",props.path);

    return (
        <li className={liClassName}>
            <a href={props.path} className={aClassName}>
                {props.name}
                {(props.path === pageURI) ? (<span className="sr-only">(current)</span>) : ''}
            </a>
        </li>
    );
};
class Navigation extends React.Component {
    constructor(props) {
        super(props);
        this.state = {str: ""};
        this.handleChange = this.handleChange.bind(this);
        this.handleSubmit = this.handleSubmit.bind(this);
    }
    handleChange(event){
        this.setState({str:event.target.value});
    }

    handleSubmit(event){
        console.log(this.props)
        this.props.history.push(`#/results/${this.state.str}`);
    }
    render() {
        if (this.props.location.hash !== '#/') {
            if (!AuthService.isAuthenticated()) {
                return (
                    <nav className="navbar navbar-expand-lg navbar-light bg-light">
                        <a className="navbar-brand" href="/">
                            <img className="img-fluid" src={logo} width={80} height={80}/>
                        </a>
                        {/*<Button className="navbar-toggler" type="button" data-toggle="collapse"*/}
                                {/*data-target="#navbarSupportedContent"*/}
                                {/*aria-controls="navbarSupportedContent" aria-expanded="false"*/}
                                {/*aria-label="Toggle navigation">*/}
                            {/*<span className="navbar-toggler-icon"></span>*/}
                        {/*</Button>*/}
                        <Button className="navbar-toggler" type="button" data-toggle="collapse"
                                data-target="#navbarSupportedContent"
                                aria-controls="navbarSupportedContent" aria-expanded="false"
                                aria-label="Toggle navigation">
                            <span className="navbar-toggler-icon"></span>
                        </Button>

                            <form className="form-inline my-2 my-lg-0" onSubmit={this.handleSubmit}>
                                <div className="search_bar_nav">
                                    <div clasName="search_nav">
                                    <input className="form-control mr-sm-2" type="search" onChange={this.handleChange}
                                           placeholder="Search for the item that you need" aria-label="Search"/>
                                    </div>
                                    <div className="Button_nav">
                                    <Button className="btn btn-outline-success my-2 my-sm-0 search_button_front_page" style={{backgroundColor:"#009900"}} type="submit">OK!</Button>
                                    </div>
                                </div>
                            </form>


                        <div className="collapse navbar-collapse" id="navbarSupportedContent">
                            <ul className="navbar-nav ml-auto">
                                <NavItem path="/" name="Become a Lessor "/>
                                <NavItem path="/" name="Earn Credit "/>
                                <NavItem path="/" name="Help"/>
                                <NavItem path="http://localhost:8000/#/login" name="Login"/>
                                <NavItem path="http://localhost:8000/#/register" name="Sign up"/>
                            </ul>
                        </div>
                    </nav>
                )
            } else {
                console.log("photo",`http://localhost:3000/photos/${this.props.user.photo}`)
                var role = AuthService.getRole();
                if (role === 'Lessee') {
                    return (
                        <nav className="navbar navbar-expand-lg navbar-light bg-light">
                            <a className="navbar-brand" href="/">
                                <img className="img-fluid" src={logo} width={80} height={80}/>
                            </a>
                            <div className="row">
                                <div className="col-9">
                                    <form onSubmit={this.handleSubmit}>
                                        <input className="form-control mr-sm-2" type="search" onChange={this.handleChange}
                                               placeholder="Search for the item that you need" aria-label="Search"/>
                                    </form>
                                </div>
                                <div className="col-3">
                                    <Button className="btn btn-outline-success my-2 my-sm-0 search_button_front_page" style={{backgroundColor:"#009900"}} type="submit">OK!</Button>
                                </div>
                            </div>
                            <div className="collapse navbar-collapse" id="navbarSupportedContent">
                                <ul className="navbar-nav ml-auto">
                                    <NavItem path="http://localhost:8000/#/become_lessor" name="Become a Lessor "/>
                                    <NavItem path="http://localhost:8000/#/my_bookings" name="Bookings"/>
                                    <NavItem path="http://localhost:8000/#/my_conversations" name="Messages"/>
                                    <NavItem path="/" name="Earn Credit"/>
                                    <NavItem path="/" name="Help"/>
                                    <NavItem path="http://localhost:8000/#/user_dashboard" name="My dashboard">
                                            {/*<span onClick={document.getElementById('id01').style.display='none'} className="close"*/}
                                            {/*title="Close Modal">&times;</span>*/}
                                        {/*<NavItem>*/}
                                            {/*<img src={`http://localhost:3000/photos/${this.props.user.photo}`}  alt="Avatar" className="avatar"/>*/}
                                        {/*</NavItem>*/}
                                    </NavItem>
                                    <NavItem path="http://localhost:8000/#/logout" name="Logout"/>
                                    {/*<KebabMenu></KebabMenu>*/}
                                </ul>
                            </div>
                        </nav>
                    )
                }
                else if (role === 'Lessor') {
                    return (
                        <nav className="navbar navbar-expand-lg navbar-light bg-light">
                            <a className="navbar-brand" href="/">
                                <img className="img-fluid" src={logo} width={80} height={80}/>
                            </a>
                            <div className="row">
                                <div className="col-9">
                                    <form onSubmit={this.handleSubmit}>
                                        <input className="form-control mr-sm-2" type="search" onChange={this.handleChange}
                                               placeholder="Search for the item that you need" aria-label="Search"/>
                                    </form>
                                </div>
                                <div className="col-3">
                                    <Button className="btn btn-outline-success my-2 my-sm-0 search_button_front_page" style={{backgroundColor:"#009900"}} type="submit">OK!</Button>
                                </div>
                            </div>
                            <div className="collapse navbar-collapse" id="navbarSupportedContent">
                                <ul className="navbar-nav ml-auto">
                                    <NavItem path="http://localhost:8000/#/add" name="Add Item"/>
                                    <NavItem path="http://localhost:8000/#/my_items" name="Items"/>
                                    <NavItem path="http://localhost:8000/#/my_bookings" name="Bookings"/>
                                    <NavItem path="http://localhost:8000/#/my_conversations" name="Messages"/>
                                    <NavItem path="/" name="Earn Credit"/>
                                    <NavItem path="/" name="Help"/>
                                    <NavItem path="http://localhost:8000/#/user_dashboard" name="My dashboard">
                                        {/*<img src={`http://localhost:3000/photos/${this.props.user.photo}`} alt="Avatar"*/}
                                             {/*className="img-fluid avatar"/>*/}
                                    </NavItem>
                                    <NavItem path="http://localhost:8000/#/logout" name="Logout"/>
                                </ul>
                            </div>
                        </nav>
                    )
                }
                else {
                    return (
                        <nav className="navbar navbar-expand-lg navbar-light bg-light">
                            <a className="navbar-brand" href="/">
                                <img className="img-fluid" src={logo} width={80} height={80}/>
                            </a>
                            <div className="row">
                                <div className="col-9">
                                    <form onSubmit={this.handleSubmit}>
                                        <input className="form-control mr-sm-2" type="search" onChange={this.handleChange}
                                               placeholder="Search for the item that you need" aria-label="Search"/>
                                    </form>
                                </div>
                                <div className="col-3">
                                    <Button className="btn btn-outline-success my-2 my-sm-0 search_button_front_page" style={{backgroundColor:"#009900"}} type="submit">OK!</Button>
                                </div>
                            </div>
                            <div className="collapse navbar-collapse" id="navbarSupportedContent">
                                <ul className="navbar-nav ml-auto">
                                    <NavItem path="http://localhost:8000/#/my_conversations" name="Messages"/>
                                    <NavItem path="http://localhost:8000/#/admin_dashboard" name="My dashboard">
                                        {/*<img src={`http://localhost:3000/photos/${this.props.user.photo}`} alt="Avatar"*/}
                                             {/*className="img-fluid avatar"/>*/}
                                    </NavItem>
                                    <NavItem path="http://localhost:8000/#/logout" name="Logout"/>
                                </ul>
                            </div>
                        </nav>
                    )
                }

            }
        }
        else{
            if (!AuthService.isAuthenticated()) {
                return (
                    <nav className="navbar navbar-expand-lg navbar-light bg-light">
                        <a className="navbar-brand" href="/">
                            <img className="img-fluid" src={logo} width={80} height={80}/>
                        </a>
                        <Button className="navbar-toggler" type="button" data-toggle="collapse"
                                data-target="#navbarSupportedContent"
                                aria-controls="navbarSupportedContent" aria-expanded="false"
                                aria-label="Toggle navigation">
                            <span className="navbar-toggler-icon"></span>
                        </Button>

                        <div className="collapse navbar-collapse" id="navbarSupportedContent">
                            <ul className="navbar-nav ml-auto">
                                <NavItem path="/" name="Become a Lessor "/>
                                <NavItem path="/" name="Earn Credit "/>
                                <NavItem path="/" name="Help"/>
                                <NavItem path="http://localhost:8000/#/login" name="Login"/>
                                <NavItem path="http://localhost:8000/#/register" name="Sign up"/>
                            </ul>
                        </div>
                    </nav>
                )
            } else {
                var role = AuthService.getRole();
                if (role === 'Lessee') {
                    return (
                        <nav className="navbar navbar-expand-lg navbar-light bg-light">
                            <a className="navbar-brand" href="/">
                                <img className="img-fluid" src={logo} width={80} height={80}/>
                            </a>
                            <Button className="navbar-toggler" type="button" data-toggle="collapse"
                                    data-target="#navbarSupportedContent"
                                    aria-controls="navbarSupportedContent" aria-expanded="false"
                                    aria-label="Toggle navigation">
                                <span className="navbar-toggler-icon"></span>
                            </Button>

                            <div className="collapse navbar-collapse" id="navbarSupportedContent">
                                <ul className="navbar-nav ml-auto">
                                    <NavItem path="http://localhost:8000/#/become_lessor" name="Become a Lessor "/>
                                    <NavItem path="http://localhost:8000/#/my_bookings" name="Bookings"/>
                                    <NavItem path="http://localhost:8000/#/my_conversations" name="Messages"/>
                                    <NavItem path="/" name="Earn Credit"/>
                                    <NavItem path="/" name="Help"/>
                                    <NavItem path="http://localhost:8000/#/user_dashboard" name="My dashboard">
                                        {/*<img src={`http://localhost:3000/photos/${this.props.user.photo}`} alt="Avatar"*/}
                                             {/*className="img-fluid avatar"/>*/}
                                    </NavItem>
                                    <NavItem path="http://localhost:8000/#/logout" name="Logout"/>
                                </ul>
                            </div>
                        </nav>
                    )
                }
                else if (role === 'Lessor') {
                    return (
                        <nav className="navbar navbar-expand-lg navbar-light bg-light">
                            <a className="navbar-brand" href="/">
                                <img className="img-fluid" src={logo} width={80} height={80}/>
                            </a>
                            <Button className="navbar-toggler" type="button" data-toggle="collapse"
                                    data-target="#navbarSupportedContent"
                                    aria-controls="navbarSupportedContent" aria-expanded="false"
                                    aria-label="Toggle navigation">
                                <span className="navbar-toggler-icon"></span>
                            </Button>

                            <div className="collapse navbar-collapse" id="navbarSupportedContent">
                                <ul className="navbar-nav ml-auto">
                                    <NavItem path="http://localhost:8000/#/add" name="Add Item"/>
                                    <NavItem path="http://localhost:8000/#/my_items" name="Items"/>
                                    <NavItem path="http://localhost:8000/#/my_bookings" name="Bookings"/>
                                    <NavItem path="http://localhost:8000/#/my_conversations" name="Messages"/>
                                    <NavItem path="/" name="Earn Credit"/>
                                    <NavItem path="/" name="Help"/>
                                    <NavItem path="http://localhost:8000/#/user_dashboard" name="My dashboard">
                                        {/*<img src={`http://localhost:3000/photos/${this.props.user.photo}`} alt="Avatar"*/}
                                             {/*className="img-fluid avatar"/>*/}
                                    </NavItem>
                                    <NavItem path="http://localhost:8000/#/logout" name="Logout"/>
                                </ul>
                            </div>
                        </nav>
                    )
                }
                else {
                    return (
                        <nav className="navbar navbar-expand-lg navbar-light bg-light">
                            <a className="navbar-brand" href="/">
                                <img className="img-fluid" src={logo} width={80} height={80}/>
                            </a>
                            <Button className="navbar-toggler" type="button" data-toggle="collapse"
                                    data-target="#navbarSupportedContent"
                                    aria-controls="navbarSupportedContent" aria-expanded="false"
                                    aria-label="Toggle navigation">
                                <span className="navbar-toggler-icon"></span>
                            </Button>

                            <div className="collapse navbar-collapse" id="navbarSupportedContent">
                                <ul className="navbar-nav ml-auto">
                                    <NavItem path="http://localhost:8000/#/my_conversations" name="Messages"/>
                                    <NavItem path="http://localhost:8000/#/admin_dashboard" name="My dashboard">
                                        {/*<img src={`http://localhost:3000/photos/${this.props.user.photo}`} alt="Avatar"*/}
                                             {/*className="img-fluid avatar"/>*/}
                                    </NavItem>
                                    <NavItem path="http://localhost:8000/#/logout" name="Logout"/>
                                </ul>
                            </div>
                        </nav>
                    )
                }

            }
        }
    }
}

export default withRouter(Navigation);